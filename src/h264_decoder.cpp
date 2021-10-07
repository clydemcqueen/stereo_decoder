#include "stereo_decoder/h264_decoder.hpp"

#include <stdexcept>

#include "sensor_msgs/image_encodings.hpp"

namespace stereo_decoder
{

H264Decoder::H264Decoder()
{
  av_init_packet(&packet_);
  av_log_set_level(AV_LOG_WARNING);

  p_codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!p_codec_) {
    throw std::runtime_error("Could not find ffmpeg h264 codec");
  }

  p_codec_context_ = avcodec_alloc_context3(p_codec_);

  if (avcodec_open2(p_codec_context_, p_codec_, nullptr) < 0) {
    throw std::runtime_error("Could not open ffmpeg h264 codec");
  }

  p_frame_ = av_frame_alloc();
}

H264Decoder::~H264Decoder()
{
  avcodec_close(p_codec_context_);
  av_free(p_codec_context_);
  av_frame_free(&p_frame_);
}

std::unique_ptr<sensor_msgs::msg::Image> H264Decoder::decode(std::shared_ptr<h264_msgs::msg::Packet> & packet)
{
  packet_.size = static_cast<int>(packet->data.size());
  packet_.data = const_cast<uint8_t *>(reinterpret_cast<uint8_t const *>(&packet->data[0]));

  // Send packet to decoder
  if (avcodec_send_packet(p_codec_context_, &packet_) < 0) {
    return nullptr;
  }

  // Get decoded frame
  // Failure to decode is common when first starting
  if (avcodec_receive_frame(p_codec_context_, p_frame_) < 0) {
    return nullptr;
  }

  auto image = std::make_unique<sensor_msgs::msg::Image>();
  image->width = p_frame_->width;
  image->height = p_frame_->height;
  image->step = 3 * p_frame_->width;
  image->encoding = sensor_msgs::image_encodings::BGR8;
  image->header = packet->header;

  // Set / update sws context
  p_sws_context_ =
    sws_getCachedContext(p_sws_context_, p_frame_->width, p_frame_->height, AV_PIX_FMT_YUV420P,
      p_frame_->width, p_frame_->height, AV_PIX_FMT_BGR24,
      SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

  // Copy and convert from YUYV420P to BGR24
  image->data.resize(p_frame_->width * p_frame_->height * 3);
  int stride = 3 * p_frame_->width;
  uint8_t * destination = &image->data[0];
  sws_scale(p_sws_context_, (const uint8_t * const *) p_frame_->data, p_frame_->linesize, 0,
    p_frame_->height, &destination, &stride);

  return image;
}

} // namespace stereo_decoder
