// io_data.h
#ifndef IO_DATA_H
#define IO_DATA_H
extern "C" {
#include <libavcodec/avcodec.h>
}
#include <stdint.h>

int32_t open_input_output_files(const char* input_name,
                                const char* output_name);
void close_input_output_files();

int32_t end_of_input_file();

int32_t read_data_to_buf(uint8_t* buf, int32_t size, int32_t& out_size);
int32_t write_frame_to_yuv(AVFrame* frame);

int32_t read_yuv_to_frame(AVFrame* frame);
void write_pkt_to_file(AVPacket* pkt);

int32_t write_samples_to_pcm(AVFrame* frame, AVCodecContext* codec_ctx);
int32_t read_pcm_to_frame(AVFrame* frame, AVCodecContext* codec_ctx);

int32_t write_samples_to_pcm2(AVFrame* frame, enum AVSampleFormat format,
                              int channels);
int32_t read_pcm_to_frame2(AVFrame* frame, enum AVSampleFormat format,
                           int channels);

void write_packed_data_to_file(const uint8_t* buf, int32_t size);
#endif