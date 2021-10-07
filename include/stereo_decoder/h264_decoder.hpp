#ifndef STEREO_DECODER_H264_DECODER_HPP
#define STEREO_DECODER_H264_DECODER_HPP

#include "h264_msgs/msg/packet.hpp"
#include "sensor_msgs/msg/image.hpp"

extern "C"
{
#include "libavcodec/avcodec.h"
#include "libswscale/swscale.h"
}

namespace stereo_decoder
{

class H264Decoder
{
  AVCodec * p_codec_{};
  AVCodecContext * p_codec_context_{};
  AVFrame * p_frame_{};
  AVPacket packet_{};
  SwsContext * p_sws_context_{};

public:
  H264Decoder();
  ~H264Decoder();
  std::unique_ptr<sensor_msgs::msg::Image> decode(std::shared_ptr<h264_msgs::msg::Packet> & packet);
};

} // namespace stereo_decoder

#endif //STEREO_DECODER_H264_DECODER_HPP
