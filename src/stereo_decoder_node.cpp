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

#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "stereo_decoder/stereo_decoder.hpp"

namespace stereo_decoder
{

// Test StereoDecoder
class StereoDecoderNode : public rclcpp::Node
{
  StereoDecoder decoder_;

  rclcpp::Subscription<h264_msgs::msg::Packet>::SharedPtr left_packet_sub_;
  rclcpp::Subscription<h264_msgs::msg::Packet>::SharedPtr right_packet_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;

  rclcpp::TimerBase::SharedPtr spin_timer_;

public:
  StereoDecoderNode()
  : Node("stereo_decoder_node")
  {
    (void) spin_timer_;
    (void) left_packet_sub_;
    (void) right_packet_sub_;

    left_image_pub_ = create_publisher<sensor_msgs::msg::Image>("stereo/left/image_raw", 10);
    right_image_pub_ = create_publisher<sensor_msgs::msg::Image>("stereo/right/image_raw", 10);

    // H.264 packets arrive at 20 fps
    left_packet_sub_ = create_subscription<h264_msgs::msg::Packet>(
      "stereo/left/image_raw/h264", 10,
      [this](h264_msgs::msg::Packet::SharedPtr msg) // NOLINT
      {
        RCLCPP_INFO(get_logger(), "push left %ld", msg->seq); // NOLINT
        decoder_.push_left(msg);
      });
    right_packet_sub_ = create_subscription<h264_msgs::msg::Packet>(
      "stereo/right/image_raw/h264", 10,
      [this](h264_msgs::msg::Packet::SharedPtr msg) // NOLINT
      {
        RCLCPP_INFO(get_logger(), "push right %ld", msg->seq); // NOLINT
        decoder_.push_right(msg);
      });

    // Publish images at 1 fps (1000ms timer)
    spin_timer_ = create_wall_timer(
      std::chrono::milliseconds{1000},
      [this]()
      {
        RCLCPP_INFO(get_logger(), "try to pop stereo pair"); // NOLINT
        std::unique_ptr<sensor_msgs::msg::Image> left_image_;
        std::unique_ptr<sensor_msgs::msg::Image> right_image_;
        if (decoder_.pop_now(left_image_, right_image_)) {
          RCLCPP_INFO(get_logger(), "publish stereo pair"); // NOLINT
          left_image_pub_->publish(std::move(left_image_));
          right_image_pub_->publish(std::move(right_image_));
        }
      });

    RCLCPP_INFO(get_logger(), "StereoDecoderNode ready");
  }
};

}  // namespace stereo_decoder

int main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<stereo_decoder::StereoDecoderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
