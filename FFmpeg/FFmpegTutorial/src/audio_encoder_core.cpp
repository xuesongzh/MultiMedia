#include "audio_encoder_core.h"

#include <stdio.h>
#include <stdlib.h>

#include <iostream>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/channel_layout.h>
#include <libavutil/common.h>
#include <libavutil/frame.h>
#include <libavutil/samplefmt.h>
}

#include "io_data.h"

static AVCodec *codec = nullptr;
static AVCodecContext *codec_ctx = nullptr;
static AVFrame *frame = nullptr;
static AVPacket *pkt = nullptr;

static enum AVCodecID audio_codec_id;

int32_t init_audio_encoder(const char *codec_name) {
  if (strcasecmp(codec_name, "MP3") == 0) {
    audio_codec_id = AV_CODEC_ID_MP3;
    std::cout << "Select codec id: MP3" << std::endl;
  } else if (strcasecmp(codec_name, "AAC") == 0) {
    audio_codec_id = AV_CODEC_ID_AAC;
    std::cout << "Select codec id: AAC" << std::endl;
  } else {
    std::cerr << "Error invalid audio format." << std::endl;
    return -1;
  }

  codec = avcodec_find_encoder(audio_codec_id);
  if (!codec) {
    std::cerr << "Error: could not find codec." << std::endl;
    return -1;
  }

  codec_ctx = avcodec_alloc_context3(codec);
  if (!codec_ctx) {
    std::cerr << "Error: could not alloc codec." << std::endl;
    return -1;
  }

  // 设置音频编码器的参数
  codec_ctx->bit_rate = 128000;                // 设置输出码率为128Kbps
  codec_ctx->sample_fmt = AV_SAMPLE_FMT_FLTP;  // 音频采样格式为fltp
  codec_ctx->sample_rate = 44100;              // 音频采样率为44.1kHz
  codec_ctx->channel_layout = AV_CH_LAYOUT_STEREO;  // 声道布局为立体声
  codec_ctx->channels = 2;                          // 声道数为双声道

  int32_t result = avcodec_open2(codec_ctx, codec, nullptr);
  if (result < 0) {
    std::cerr << "Error: could not open codec." << std::endl;
    return -1;
  }

  frame = av_frame_alloc();
  if (!frame) {
    std::cerr << "Error: could not alloc frame." << std::endl;
    return -1;
  }

  frame->nb_samples = codec_ctx->frame_size;
  frame->format = codec_ctx->sample_fmt;
  frame->channel_layout = codec_ctx->channel_layout;
  result = av_frame_get_buffer(frame, 0);
  if (result < 0) {
    std::cerr << "Error: AVFrame could not get buffer." << std::endl;
    return -1;
  }

  pkt = av_packet_alloc();
  if (!pkt) {
    std::cerr << "Error: could not alloc packet." << std::endl;
    return -1;
  }
  return 0;
}

static int32_t encode_frame(bool flushing) {
  int32_t result = 0;
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
    std::cout << "received pkt size:" << pkt->size << std::endl;
    write_pkt_to_file(pkt);
  }
  return 0;
}

int32_t audio_encoding() {
  int32_t result = 0;
  while (!end_of_input_file()) {
    result = read_pcm_to_frame(frame, codec_ctx);
    if (result < 0) {
      std::cerr << "Error: read_pcm_to_frame failed." << std::endl;
      return -1;
    }

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

void destroy_audio_encoder() {
  av_frame_free(&frame);
  av_packet_free(&pkt);
  avcodec_free_context(&codec_ctx);
}
