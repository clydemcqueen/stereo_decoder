// MIT License
//
// Copyright (c) 2021 Clyde McQueen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <iostream>
#include <memory>
#include <utility>

#include "stereo_decoder/stereo_decoder.hpp"

namespace stereo_decoder
{

StereoDecoder::StereoDecoder()
{
  // Could move thread to a Worker class one day...
  left_thread_ = std::thread(
    [this]()
    {
      while (true) {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] {return stop_signal_ || !left_input_.empty();});

        if (stop_signal_) {
          break;
        }

        auto packet = left_input_.front();
        left_input_.pop();
        lock.unlock();
        condition_.notify_all();

        auto image = left_decoder_.decode(packet);

        if (image) {
          lock.lock();
          left_output_ = std::move(image);
          lock.unlock();
          condition_.notify_all();
        }
      }

      std::cout << "stopping left thread" << std::endl;
    });

  right_thread_ = std::thread(
    [this]()
    {
      while (true) {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] {return stop_signal_ || !right_input_.empty();});

        if (stop_signal_) {
          break;
        }

        auto packet = right_input_.front();
        right_input_.pop();
        lock.unlock();
        condition_.notify_all();

        auto image = right_decoder_.decode(packet);

        if (image) {
          lock.lock();
          right_output_ = std::move(image);
          lock.unlock();
          condition_.notify_all();
        }
      }

      std::cout << "stopping right thread" << std::endl;
    });
}

StereoDecoder::~StereoDecoder()
{
  stop();
}

void StereoDecoder::push_left(const h264_msgs::msg::Packet::SharedPtr & msg)
{
  std::unique_lock<std::mutex> lock(mutex_);

  if (stop_signal_) {
    return;
  }

  left_input_.push(msg);
  lock.unlock();
  condition_.notify_all();
}

void StereoDecoder::push_right(const h264_msgs::msg::Packet::SharedPtr & msg)
{
  std::unique_lock<std::mutex> lock(mutex_);

  if (stop_signal_) {
    return;
  }

  right_input_.push(msg);
  lock.unlock();
  condition_.notify_all();
}

bool StereoDecoder::pop_now(
  std::unique_ptr<sensor_msgs::msg::Image> & left,
  std::unique_ptr<sensor_msgs::msg::Image> & right)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (stop_signal_ || !left_output_ || !right_output_) {
    return false;
  } else {
    left = std::move(left_output_);
    right = std::move(right_output_);
    return true;
  }
}

bool StereoDecoder::pop_wait(
  std::unique_ptr<sensor_msgs::msg::Image> & left,
  std::unique_ptr<sensor_msgs::msg::Image> & right)
{
  std::unique_lock<std::mutex> lock(mutex_);
  condition_.wait(lock, [this] {return stop_signal_ || (left_output_ && right_output_);});

  if (stop_signal_) {
    return false;
  } else {
    left = std::move(left_output_);
    right = std::move(right_output_);
    return true;
  }
}

void StereoDecoder::stop()
{
  std::unique_lock<std::mutex> lock(mutex_);
  stop_signal_ = true;
  std::cout << "stopping" << std::endl;
  lock.unlock();
  condition_.notify_all();

  if (left_thread_.joinable()) {
    left_thread_.join();
  }

  if (right_thread_.joinable()) {
    right_thread_.join();
  }
}

}  // namespace stereo_decoder
