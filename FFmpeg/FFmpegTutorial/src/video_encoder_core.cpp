// video_encoder_core.cpp
#include "video_encoder_core.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
}
#include <iostream>
#include <string>

#include "io_data.h"

static AVCodec *codec = nullptr;
static AVCodecContext *codec_ctx = nullptr;
static AVFrame *frame = nullptr;
static AVPacket *pkt = nullptr;

int32_t init_video_encoder(const char *codec_name) {
  // 验证输入编码器名称非空
  if (strlen(codec_name) == 0) {
    std::cerr << "Error: empty codec name." << std::endl;
    return -1;
  }

  // 查找编码器
  codec = avcodec_find_encoder_by_name(codec_name);
  if (!codec) {
    std::cerr << "Error: could not find codec with codec name:"
              << std::string(codec_name) << std::endl;
    return -1;
  }

  // 创建编码器上下文结构
  codec_ctx = avcodec_alloc_context3(codec);
  if (!codec_ctx) {
    std::cerr << "Error: could not allocate video codec context." << std::endl;
    return -1;
  }

  // 配置编码参数
  codec_ctx->profile = FF_PROFILE_H264_HIGH;
  codec_ctx->bit_rate = 2000000;
  codec_ctx->width = 1280;
  codec_ctx->height = 720;
  codec_ctx->gop_size = 10;
  codec_ctx->time_base = (AVRational){1, 25};
  codec_ctx->framerate = (AVRational){25, 1};
  codec_ctx->max_b_frames = 3;
  codec_ctx->pix_fmt = AV_PIX_FMT_YUV420P;

  if (codec->id == AV_CODEC_ID_H264) {
    av_opt_set(codec_ctx->priv_data, "preset", "slow", 0);
  }

  // 使用指定的 codec 初始化编码器上下文结构
  int32_t result = avcodec_open2(codec_ctx, codec, nullptr);
  if (result < 0) {
    std::cerr << "Error: could not open codec:"
              << std::string(av_err2str(result)) << std::endl;
    return -1;
  }

  pkt = av_packet_alloc();
  if (!pkt) {
    std::cerr << "Error: could not allocate AVPacket." << std::endl;
    return -1;
  }

  frame = av_frame_alloc();
  if (!frame) {
    std::cerr << "Error: could not allocate AVFrame." << std::endl;
    return -1;
  }
  frame->width = codec_ctx->width;
  frame->height = codec_ctx->height;
  frame->format = codec_ctx->pix_fmt;

  result = av_frame_get_buffer(frame, 0);
  if (result < 0) {
    std::cerr << "Error: could not get AVFrame buffer." << std::endl;
    return -1;
  }

  return 0;
}

void destroy_video_encoder() {
  // 释放编码器上下文结构
  avcodec_free_context(&codec_ctx);
  // 释放 Frame 和 Packet 结构
  av_frame_free(&frame);
  av_packet_free(&pkt);
}

static int32_t encode_frame(bool flushing) {
  int32_t result = 0;
  if (!flushing) {
    std::cout << "Send frame to encoder with pts: " << frame->pts << std::endl;
  }

  result = avcodec_send_frame(codec_ctx, flushing ? nullptr : frame);
  if (result < 0) {
    std::cerr << "Error: avcodec_send_frame failed." << std::endl;
    return result;
  }

  while (result >= 0) {
    result = avcodec_receive_packet(codec_ctx, pkt);
    if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
      return 1;
    } else if (result < 0) {
      std::cerr << "Error: avcodec_receive_packet failed." << std::endl;
      return result;
    }

    if (flushing) {
      std::cout << "Flushing:";
    }
    std::cout << "Got encoded package with dts:" << pkt->dts
              << ", pts:" << pkt->pts << ", " << std::endl;
    write_pkt_to_file(pkt);
  }
  return 0;
}

int32_t encoding(int32_t frame_cnt) {
  int result = 0;
  for (size_t i = 0; i < frame_cnt; i++) {
    result = av_frame_make_writable(frame);
    if (result < 0) {
      std::cerr << "Error: could not av_frame_make_writable." << std::endl;
      return result;
    }

    result = read_yuv_to_frame(frame);
    if (result < 0) {
      std::cerr << "Error: read_yuv_to_frame failed." << std::endl;
      return result;
    }
    frame->pts = i;

    result = encode_frame(false);
    if (result < 0) {
      std::cerr << "Error: encode_frame failed." << std::endl;
      return result;
    }
  }
  result = encode_frame(true);
  if (result < 0) {
    std::cerr << "Error: flushing failed." << std::endl;
    return result;
  }

  return 0;
}