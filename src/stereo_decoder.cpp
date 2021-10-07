#include <iostream>
#include "stereo_decoder/stereo_decoder.hpp"

namespace stereo_decoder
{

StereoDecoder::StereoDecoder()
{
  // TODO -> Worker class
  left_thread_ = std::thread([this]()
  {
    while (!stop_signal_) {
      std::unique_lock<std::mutex> left_input_lock(left_input_mutex_);
      left_input_condition_.wait(left_input_lock, [this] { return stop_signal_ || !left_input_.empty(); });

      if (stop_signal_) {
        break;
      }

      auto packet = left_input_.front();
      left_input_.pop();
      left_input_lock.unlock();
      left_input_condition_.notify_one();

      auto image = left_decoder_.decode(packet);

      if (image) {
        std::unique_lock<std::mutex> output_lock(output_mutex_);
        left_output_ = std::move(image);
        output_lock.unlock();
      }
    }
  });

  right_thread_ = std::thread([this]()
  {
    while (!stop_signal_) {
      std::unique_lock<std::mutex> right_input_lock(right_input_mutex_);
      right_input_condition_.wait(right_input_lock, [this] { return stop_signal_ || !right_input_.empty(); });

      if (stop_signal_) {
        break;
      }
      auto packet = right_input_.front();
      right_input_.pop();
      right_input_lock.unlock();

      auto image = right_decoder_.decode(packet);

      if (image) {
        std::unique_lock<std::mutex> output_lock(output_mutex_);
        right_output_ = std::move(image);
        output_lock.unlock();
        output_condition_.notify_one();
      }
    }
  });
}

StereoDecoder::~StereoDecoder()
{
  // Set stop signal, then wake up threads
  stop_signal_ = true;

  // TODO -> Worker class
  if (left_thread_.joinable()) {
    left_input_condition_.notify_one();
    left_thread_.join();
  }

  if (right_thread_.joinable()) {
    right_input_condition_.notify_one();
    right_thread_.join();
  }
}

void StereoDecoder::push_left(const h264_msgs::msg::Packet::SharedPtr & msg)
{
  std::unique_lock<std::mutex> left_input_lock(left_input_mutex_);
  left_input_.push(msg);
  left_input_lock.unlock();
  left_input_condition_.notify_one();
}

void StereoDecoder::push_right(const h264_msgs::msg::Packet::SharedPtr & msg)
{
  std::unique_lock<std::mutex> right_input_lock(right_input_mutex_);
  right_input_.push(msg);
  right_input_lock.unlock();
  right_input_condition_.notify_one();
}

bool StereoDecoder::pop_now(
  std::unique_ptr<sensor_msgs::msg::Image> & left,
  std::unique_ptr<sensor_msgs::msg::Image> & right)
{
  std::lock_guard<std::mutex> output_lock(output_mutex_);
  if (left_output_ && right_output_) {
    left = std::move(left_output_);
    right = std::move(right_output_);
    return true;
  } else {
    return false;
  }
}

bool StereoDecoder::pop_wait(
  std::unique_ptr<sensor_msgs::msg::Image> & left,
  std::unique_ptr<sensor_msgs::msg::Image> & right)
{
  std::unique_lock<std::mutex> output_lock(output_mutex_);
  output_condition_.wait(output_lock, [this] { return stop_signal_ || (left_output_ && right_output_); });

  if (stop_signal_) {
    return false;
  } else {
    left = std::move(left_output_);
    right = std::move(right_output_);
    return true;
  }
}

} // namespace stereo_decoder
