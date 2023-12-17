// io_data.cpp
#include "io_data.h"

#include <stdlib.h>
#include <string.h>

#include <iostream>

static FILE* input_file = nullptr;
static FILE* output_file = nullptr;

int32_t open_input_output_files(const char* input_name,
                                const char* output_name) {
  if (strlen(input_name) == 0 || strlen(output_name) == 0) {
    std::cerr << "Error: empty input or output file name." << std::endl;
    return -1;
  }

  close_input_output_files();

  input_file = fopen(input_name, "rb");
  if (input_file == nullptr) {
    std::cerr << "Error: failed to open input file." << std::endl;
    return -1;
  }
  output_file = fopen(output_name, "wb");
  if (output_file == nullptr) {
    std::cerr << "Error: failed to open output file." << std::endl;
    return -1;
  }

  return 0;
}

void close_input_output_files() {
  if (input_file != nullptr) {
    fclose(input_file);
    input_file = nullptr;
  }
  if (output_file != nullptr) {
    fclose(output_file);
    output_file = nullptr;
  }
}

int32_t end_of_input_file() { return feof(input_file); }

int32_t read_data_to_buf(uint8_t* buf, int32_t size, int32_t& out_size) {
  int32_t read_size = fread(buf, 1, size, input_file);
  if (read_size == 0) {
    std::cerr << "Error: read_data_to_buf failed." << std::endl;
    return -1;
  }
  out_size = read_size;
  return 0;
}

int32_t write_frame_to_yuv(AVFrame* frame) {
  uint8_t** pBuf = frame->data;
  int* pStride = frame->linesize;
  for (size_t i = 0; i < 3; i++) {
    int32_t width = (i == 0 ? frame->width : frame->width / 2);
    int32_t height = (i == 0 ? frame->height : frame->height / 2);
    for (size_t j = 0; j < height; j++) {
      fwrite(pBuf[i], 1, width, output_file);
      pBuf[i] += pStride[i];
    }
  }
  return 0;
}

int32_t read_yuv_to_frame(AVFrame* frame) {
  int32_t frame_width = frame->width;
  int32_t frame_height = frame->height;
  int32_t luma_stride = frame->linesize[0];
  int32_t chroma_stride = frame->linesize[1];
  int32_t frame_size = frame_width * frame_height * 3 / 2;
  int32_t read_size = 0;

  if (frame_width == luma_stride) {
    // 如果 width 等于 stride，说明 frame 中不存在 padding
    // 字节，可向其中整体读取
    read_size +=
        fread(frame->data[0], 1, frame_width * frame_height, input_file);
    read_size +=
        fread(frame->data[1], 1, frame_width * frame_height / 4, input_file);
    read_size +=
        fread(frame->data[2], 1, frame_width * frame_height / 4, input_file);
  } else {
    //  如果 width 不等于 stride，说明 frame 一行的末尾存在 padding
    //  字节，三个分量都应当逐行读取
    for (size_t i = 0; i < frame_height; i++) {
      read_size +=
          fread(frame->data[0] + i * luma_stride, 1, frame_width, input_file);
    }
    for (size_t uv = 1; uv < 2; uv++) {
      for (size_t i = 0; i < frame_height / 2; i++) {
        read_size += fread(frame->data[uv] + i * chroma_stride, 1,
                           frame_width / 2, input_file);
      }
    }
  }

  // 验证读取数据正确
  if (read_size != frame_size) {
    std::cerr << "Error: Read data error, frame_size:" << frame_size
              << ", read_size:" << read_size << std::endl;
    return -1;
  }

  return 0;
}

void write_pkt_to_file(AVPacket* pkt) {
  fwrite(pkt->data, 1, pkt->size, output_file);
}

int32_t write_samples_to_pcm(AVFrame* frame, AVCodecContext* codec_ctx) {
  int data_size = av_get_bytes_per_sample(codec_ctx->sample_fmt);
  if (data_size < 0) {
    /* This should not occur, checking just for paranoia */
    std::cerr << "Failed to calculate data size" << std::endl;
    exit(1);
  }
  for (int i = 0; i < frame->nb_samples; i++) {
    for (int ch = 0; ch < codec_ctx->channels; ch++) {
      fwrite(frame->data[ch] + data_size * i, 1, data_size, output_file);
    }
  }
  return 0;
}

int32_t read_pcm_to_frame(AVFrame* frame, AVCodecContext* codec_ctx) {
  int data_size = av_get_bytes_per_sample(codec_ctx->sample_fmt);
  if (data_size < 0) {
    /* This should not occur, checking just for paranoia */
    std::cerr << "Failed to calculate data size" << std::endl;
    return -1;
  }

  // 从输入文件中交替读取一个采样值的各个声道的数据，
  // 保存到AVFrame结构的存储分量中
  for (int i = 0; i < frame->nb_samples; i++) {
    for (int ch = 0; ch < codec_ctx->channels; ch++) {
      fread(frame->data[ch] + data_size * i, 1, data_size, input_file);
    }
  }
  return 0;
}

int32_t write_samples_to_pcm2(AVFrame* frame, enum AVSampleFormat format,
                              int channels) {
  int data_size = av_get_bytes_per_sample(format);
  if (data_size < 0) {
    /* This should not occur, checking just for paranoia */
    std::cerr << "Failed to calculate data size" << std::endl;
    exit(1);
  }
  for (int i = 0; i < frame->nb_samples; i++) {
    for (int ch = 0; ch < channels; ch++) {
      fwrite(frame->data[ch] + data_size * i, 1, data_size, output_file);
    }
  }
  return 0;
}

int32_t read_pcm_to_frame2(AVFrame* frame, enum AVSampleFormat format,
                           int channels) {
  int data_size = av_get_bytes_per_sample(format);
  if (data_size < 0) {
    /* This should not occur, checking just for paranoia */
    std::cerr << "Failed to calculate data size" << std::endl;
    return -1;
  }

  // 从输入文件中交替读取一个采样值的各个声道的数据，
  // 保存到AVFrame结构的存储分量中
  for (int i = 0; i < frame->nb_samples; i++) {
    for (int ch = 0; ch < channels; ch++) {
      fread(frame->data[ch] + data_size * i, 1, data_size, input_file);
    }
  }
  return 0;
}

void write_packed_data_to_file(const uint8_t* buf, int32_t size) {
  fwrite(buf, 1, size, output_file);
}