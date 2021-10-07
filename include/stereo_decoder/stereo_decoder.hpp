#ifndef STEREO_DECODER_STEREO_DECODER_HPP
#define STEREO_DECODER_STEREO_DECODER_HPP

#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <atomic>

#include "stereo_decoder/h264_decoder.hpp"

namespace stereo_decoder
{

// Multi-threaded stereo H.264 decoder. Ignores timestamps, doesn't use ApproxSync, just keeps the last output.
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
  std::atomic<bool> stop_signal_{false};

  std::mutex left_input_mutex_;
  std::condition_variable left_input_condition_;

  std::mutex right_input_mutex_;
  std::condition_variable right_input_condition_;

  std::mutex output_mutex_;
  std::condition_variable output_condition_;

public:
  StereoDecoder();
  ~StereoDecoder();

  // Add a H.264 packet to the input queue
  void push_left(const h264_msgs::msg::Packet::SharedPtr & msg);
  void push_right(const h264_msgs::msg::Packet::SharedPtr & msg);

  // If there is a stereo pair, consume it and return true
  bool pop_now(std::unique_ptr<sensor_msgs::msg::Image> & left, std::unique_ptr<sensor_msgs::msg::Image> & right);

  // Wait for a stereo pair
  // TODO the API doesn't quite work for this case... need a shutdown() or stop() method
  // TODO all other calls should check stop_signal_ and return immediately
  bool pop_wait(std::unique_ptr<sensor_msgs::msg::Image> & left, std::unique_ptr<sensor_msgs::msg::Image> & right);
};

} // namespace stereo_decoder

#endif //STEREO_DECODER_STEREO_DECODER_HPP