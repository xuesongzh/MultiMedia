#include "audio_resampler_core.h"

#include <stdlib.h>
#include <string.h>

#include <iostream>

#include "io_data.h"

extern "C" {
#include <libavutil/channel_layout.h>
#include <libavutil/frame.h>
#include <libavutil/opt.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
}

#define SRC_NB_SAMPLES 1152

static struct SwrContext *swr_ctx;
static AVFrame *input_frame = nullptr;
int32_t dst_nb_samples, max_dst_nb_samples, dst_nb_channels, dst_rate, src_rate;
enum AVSampleFormat src_sample_fmt = AV_SAMPLE_FMT_NONE,
                    dst_sample_fmt = AV_SAMPLE_FMT_NONE;
uint8_t **dst_data = NULL;
int32_t dst_linesize = 0;

static int32_t init_frame(int sample_rate, int sample_format,
                          uint64_t channel_layout) {
  int32_t result = 0;
  input_frame->sample_rate = sample_rate;
  input_frame->nb_samples = SRC_NB_SAMPLES;
  input_frame->format = sample_format;
  input_frame->channel_layout = channel_layout;

  result = av_frame_get_buffer(input_frame, 0);
  if (result < 0) {
    std::cerr << "Error: AVFrame could not get buffer." << std::endl;
    return -1;
  }

  return result;
}

int32_t init_audio_resampler(int32_t in_sample_rate, const char *in_sample_fmt,
                             const char *in_ch_layout, int32_t out_sample_rate,
                             const char *out_sample_fmt,
                             const char *out_ch_layout) {
  int32_t result = 0;
  swr_ctx = swr_alloc();
  if (!swr_ctx) {
    std::cerr << "Error: failed to allocate SwrContext." << std::endl;
    return -1;
  }

  int64_t src_ch_layout = -1, dst_ch_layout = -1;
  if (!strcasecmp(in_ch_layout, "MONO")) {
    src_ch_layout = AV_CH_LAYOUT_MONO;
  } else if (!strcasecmp(in_ch_layout, "STEREO")) {
    src_ch_layout = AV_CH_LAYOUT_STEREO;
  } else if (!strcasecmp(in_ch_layout, "SURROUND")) {
    src_ch_layout = AV_CH_LAYOUT_SURROUND;
  } else {
    std::cerr << "Error: unsupported input channel layout." << std::endl;
    return -1;
  }
  if (!strcasecmp(out_ch_layout, "MONO")) {
    dst_ch_layout = AV_CH_LAYOUT_MONO;
  } else if (!strcasecmp(out_ch_layout, "STEREO")) {
    dst_ch_layout = AV_CH_LAYOUT_STEREO;
  } else if (!strcasecmp(out_ch_layout, "SURROUND")) {
    dst_ch_layout = AV_CH_LAYOUT_SURROUND;
  } else {
    std::cerr << "Error: unsupported output channel layout." << std::endl;
    return -1;
  }

  if (!strcasecmp(in_sample_fmt, "fltp")) {
    src_sample_fmt = AV_SAMPLE_FMT_FLTP;
  } else if (!strcasecmp(in_sample_fmt, "s16")) {
    src_sample_fmt = AV_SAMPLE_FMT_S16P;
  } else {
    std::cerr << "Error: unsupported input sample format." << std::endl;
    return -1;
  }
  if (!strcasecmp(out_sample_fmt, "fltp")) {
    dst_sample_fmt = AV_SAMPLE_FMT_FLTP;
  } else if (!strcasecmp(out_sample_fmt, "s16")) {
    dst_sample_fmt = AV_SAMPLE_FMT_S16P;
  } else {
    std::cerr << "Error: unsupported output sample format." << std::endl;
    return -1;
  }

  src_rate = in_sample_rate;
  dst_rate = out_sample_rate;
  av_opt_set_int(swr_ctx, "in_channel_layout", src_ch_layout, 0);
  av_opt_set_int(swr_ctx, "in_sample_rate", src_rate, 0);
  av_opt_set_sample_fmt(swr_ctx, "in_sample_fmt", src_sample_fmt, 0);

  av_opt_set_int(swr_ctx, "out_channel_layout", dst_ch_layout, 0);
  av_opt_set_int(swr_ctx, "out_sample_rate", dst_rate, 0);
  av_opt_set_sample_fmt(swr_ctx, "out_sample_fmt", dst_sample_fmt, 0);

  result = swr_init(swr_ctx);
  if (result < 0) {
    std::cerr << "Error: failed to initialize SwrContext." << std::endl;
    return -1;
  }

  input_frame = av_frame_alloc();
  if (!input_frame) {
    std::cerr << "Error: could not alloc input frame." << std::endl;
    return -1;
  }
  result = init_frame(in_sample_rate, src_sample_fmt, src_ch_layout);
  if (result < 0) {
    std::cerr << "Error: failed to initialize input frame." << std::endl;
    return -1;
  }
  max_dst_nb_samples = dst_nb_samples = av_rescale_rnd(
      SRC_NB_SAMPLES, out_sample_rate, in_sample_rate, AV_ROUND_UP);
  dst_nb_channels = av_get_channel_layout_nb_channels(dst_ch_layout);
  std::cout << "max_dst_nb_samples:" << max_dst_nb_samples
            << ", dst_nb_channels : " << dst_nb_channels << std::endl;

  return result;
}

static int32_t resampling_frame() {
  int32_t result = 0;
  int32_t dst_bufsize = 0;
  dst_nb_samples =
      av_rescale_rnd(swr_get_delay(swr_ctx, src_rate) + SRC_NB_SAMPLES,
                     dst_rate, src_rate, AV_ROUND_UP);
  if (dst_nb_samples > max_dst_nb_samples) {
    av_freep(&dst_data[0]);
    result = av_samples_alloc(dst_data, &dst_linesize, dst_nb_channels,
                              dst_nb_samples, dst_sample_fmt, 1);
    if (result < 0) {
      std::cerr << "Error:failed to reallocat dst_data." << std::endl;
      return -1;
    }
    std::cout << "nb_samples exceeds max_dst_nb_samples, buffer reallocated."
              << std::endl;
    max_dst_nb_samples = dst_nb_samples;
  }
  result = swr_convert(swr_ctx, dst_data, dst_nb_samples,
                       (const uint8_t **)input_frame->data, SRC_NB_SAMPLES);
  if (result < 0) {
    std::cerr << "Error:swr_convert failed." << std::endl;
    return -1;
  }
  dst_bufsize = av_samples_get_buffer_size(&dst_linesize, dst_nb_channels,
                                           result, dst_sample_fmt, 1);
  if (dst_bufsize < 0) {
    std::cerr << "Error:Could not get sample buffer size." << std::endl;
    return -1;
  }
  write_packed_data_to_file(dst_data[0], dst_bufsize);

  return result;
}

int32_t audio_resampling() {
  int32_t result = av_samples_alloc_array_and_samples(
      &dst_data, &dst_linesize, dst_nb_channels, dst_nb_samples, dst_sample_fmt,
      0);
  if (result < 0) {
    std::cerr << "Error: av_samples_alloc_array_and_samples failed."
              << std::endl;
    return -1;
  }
  std::cout << "dst_linesize:" << dst_linesize << std::endl;

  while (!end_of_input_file()) {
    result = read_pcm_to_frame2(input_frame, src_sample_fmt, 2);
    if (result < 0) {
      std::cerr << "Error: read_pcm_to_frame failed." << std::endl;
      return -1;
    }
    result = resampling_frame();
    if (result < 0) {
      std::cerr << "Error: resampling_frame failed." << std::endl;
      return -1;
    }
  }

  return result;
}

void destroy_audio_resampler() {
  av_frame_free(&input_frame);
  if (dst_data) av_freep(&dst_data[0]);
  av_freep(&dst_data);
  swr_free(&swr_ctx);
}
