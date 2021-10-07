#ifndef STEREO_DECODER_STEREO_DECODER_HPP
#define STEREO_DECODER_STEREO_DECODER_HPP

#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>

#include "stereo_decoder/h264_decoder.hpp"

namespace stereo_decoder
{

// Multi-threaded stereo H.264 decoder. Ignore timestamps, don't use ApproxSync, keep the last output.
// This works pretty well because the camera rate is >> the SLAM rate. Be sure to process all H.264 packets.
class StereoDecoder
{
  std::queue<std::shared_ptr<h264_msgs::msg::Packet>> left_input_;
  std::queue<std::shared_ptr<h264_msgs::msg::Packet>> right_input_;

  std::unique_ptr<sensor_msgs::msg::Image> left_output_;
  std::unique_ptr<sensor_msgs::msg::Image> right_output_;

  H264Decoder left_decoder_;
  H264Decoder right_decoder_;

  std::thread left_thread_;
  std::thread right_thread_;

  std::mutex mutex_;
  std::condition_variable condition_;
  bool stop_signal_{false};

public:
  StereoDecoder();
  ~StereoDecoder();

  // Add a H.264 packet to the input queue
  void push_left(const h264_msgs::msg::Packet::SharedPtr & msg);
  void push_right(const h264_msgs::msg::Packet::SharedPtr & msg);

  // If there is a stereo pair, consume it and return true
  bool pop_now(std::unique_ptr<sensor_msgs::msg::Image> & left, std::unique_ptr<sensor_msgs::msg::Image> & right);

  // Wait for a stereo pair, return false if the decoder has stopped
  bool pop_wait(std::unique_ptr<sensor_msgs::msg::Image> & left, std::unique_ptr<sensor_msgs::msg::Image> & right);

  // Stop the decoder
  void stop();
};

} // namespace stereo_decoder

#endif //STEREO_DECODER_STEREO_DECODER_HPP
