#include "audio_filter_core.h"

#include <stdlib.h>
#include <string.h>

#include <iostream>

#include "audio_filter_core.h"
#include "io_data.h"

extern "C" {
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavutil/frame.h>
#include <libavutil/opt.h>

#include "libavfilter/avfilter.h"
#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/samplefmt.h"
}

#define INPUT_SAMPLERATE 44100
#define INPUT_FORMAT AV_SAMPLE_FMT_FLTP
#define INPUT_CHANNEL_LAYOUT AV_CH_LAYOUT_STEREO
#define FRAME_SIZE 4096

static AVFilterGraph *filter_graph;
static AVFilterContext *abuffersrc_ctx;
static AVFilterContext *volume_ctx;
static AVFilterContext *aformat_ctx;
static AVFilterContext *abuffersink_ctx;

static AVFrame *input_frame = nullptr, *output_frame = nullptr;

int32_t init_audio_filter(char *volume_factor) {
  int32_t result = 0;
  char ch_layout[64];
  char options_str[1024];
  AVDictionary *options_dict = NULL;

  /* 创建滤镜图 */
  filter_graph = avfilter_graph_alloc();
  if (!filter_graph) {
    std::cout << "Error: Unable to create filter graph." << std::endl;
    return AVERROR(ENOMEM);
  }

  /* 创建abuffer滤镜 */
  const AVFilter *abuffer = avfilter_get_by_name("abuffer");
  if (!abuffer) {
    std::cout << "Error: Could not find the abuffer filter." << std::endl;
    return AVERROR_FILTER_NOT_FOUND;
  }

  abuffersrc_ctx = avfilter_graph_alloc_filter(filter_graph, abuffer, "src");
  if (!abuffersrc_ctx) {
    std::cout << "Error: Could not allocate the abuffer instance." << std::endl;
    return AVERROR(ENOMEM);
  }

  av_get_channel_layout_string(ch_layout, sizeof(ch_layout), 0,
                               INPUT_CHANNEL_LAYOUT);
  av_opt_set(abuffersrc_ctx, "channel_layout", ch_layout,
             AV_OPT_SEARCH_CHILDREN);
  av_opt_set(abuffersrc_ctx, "sample_fmt", av_get_sample_fmt_name(INPUT_FORMAT),
             AV_OPT_SEARCH_CHILDREN);
  av_opt_set_q(abuffersrc_ctx, "time_base", (AVRational){1, INPUT_SAMPLERATE},
               AV_OPT_SEARCH_CHILDREN);
  av_opt_set_int(abuffersrc_ctx, "sample_rate", INPUT_SAMPLERATE,
                 AV_OPT_SEARCH_CHILDREN);

  result = avfilter_init_str(abuffersrc_ctx, NULL);
  if (result < 0) {
    std::cout << "Error: Could not initialize the abuffer filter." << std::endl;
    return result;
  }

  /* 创建volumn滤镜 */
  const AVFilter *volume = avfilter_get_by_name("volume");
  if (!volume) {
    std::cout << "Error: Could not find the volumn filter." << std::endl;
    return AVERROR_FILTER_NOT_FOUND;
  }

  volume_ctx = avfilter_graph_alloc_filter(filter_graph, volume, "volume");
  if (!volume_ctx) {
    std::cout << "Error: Could not allocate the volume instance." << std::endl;
    return AVERROR(ENOMEM);
  }

  av_dict_set(&options_dict, "volume", volume_factor, 0);
  result = avfilter_init_dict(volume_ctx, &options_dict);
  av_dict_free(&options_dict);
  if (result < 0) {
    std::cout << "Error: Could not initialize the volume filter." << std::endl;
    return result;
  }

  /* 创建aformat滤镜 */
  const AVFilter *aformat = avfilter_get_by_name("aformat");
  if (!aformat) {
    std::cout << "Error: Could not find the aformat filter." << std::endl;
    return AVERROR_FILTER_NOT_FOUND;
  }

  aformat_ctx = avfilter_graph_alloc_filter(filter_graph, aformat, "aformat");
  if (!aformat_ctx) {
    std::cout << "Error: Could not allocate the aformat instance." << std::endl;
    return AVERROR(ENOMEM);
  }

  snprintf(options_str, sizeof(options_str),
           "sample_fmts=%s:sample_rates=%d:channel_layouts=0x%" PRIx64,
           av_get_sample_fmt_name(AV_SAMPLE_FMT_S16), 22050,
           (uint64_t)AV_CH_LAYOUT_MONO);
  result = avfilter_init_str(aformat_ctx, options_str);
  if (result < 0) {
    std::cout << "Error: Could not initialize the aformat filter." << std::endl;
    return result;
  }

  /* 创建abuffersink滤镜 */
  const AVFilter *abuffersink = avfilter_get_by_name("abuffersink");
  if (!abuffersink) {
    std::cout << "Error: Could not find the abuffersink filter." << std::endl;
    return AVERROR_FILTER_NOT_FOUND;
  }

  abuffersink_ctx =
      avfilter_graph_alloc_filter(filter_graph, abuffersink, "sink");
  if (!abuffersink_ctx) {
    std::cout << "Error: Could not allocate the abuffersink instance."
              << std::endl;
    return AVERROR(ENOMEM);
  }

  result = avfilter_init_str(abuffersink_ctx, NULL);
  if (result < 0) {
    std::cout << "Error: Could not initialize the abuffersink instance."
              << std::endl;
    return result;
  }

  /* 连接创建好的滤镜 */
  result = avfilter_link(abuffersrc_ctx, 0, volume_ctx, 0);
  if (result >= 0) result = avfilter_link(volume_ctx, 0, aformat_ctx, 0);
  if (result >= 0) result = avfilter_link(aformat_ctx, 0, abuffersink_ctx, 0);
  if (result < 0) {
    fprintf(stderr, "Error connecting filters\n");
    return result;
  }

  /* 配置滤镜图 */
  result = avfilter_graph_config(filter_graph, NULL);
  if (result < 0) {
    std::cout << "Error: Error configuring the filter graph." << std::endl;
    return result;
  }

  /* 创建输入帧对象和输出帧对象 */
  input_frame = av_frame_alloc();
  if (!input_frame) {
    std::cerr << "Error: could not alloc input frame." << std::endl;
    return -1;
  }

  output_frame = av_frame_alloc();
  if (!output_frame) {
    std::cerr << "Error: could not alloc input frame." << std::endl;
    return -1;
  }

  return result;
}

static int32_t filter_frame() {
  int32_t result = av_buffersrc_add_frame(abuffersrc_ctx, input_frame);
  if (result < 0) {
    std::cerr << "Error:add frame to buffersrc failed." << std::endl;
    return result;
  }

  while (1) {
    result = av_buffersink_get_frame(abuffersink_ctx, output_frame);
    if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
      return 1;
    } else if (result < 0) {
      std::cerr << "Error: buffersink_get_frame failed." << std::endl;
      return result;
    }
    std::cout << "Output channels:" << output_frame->channels
              << ", nb_samples : " << output_frame->nb_samples
              << ", sample_fmt : " << output_frame->format << std::endl;
    write_samples_to_pcm2(output_frame, (AVSampleFormat)output_frame->format,
                          output_frame->channels);
    av_frame_unref(output_frame);
  }

  return result;
}

static int32_t init_frame() {
  input_frame->sample_rate = INPUT_SAMPLERATE;
  input_frame->nb_samples = FRAME_SIZE;
  input_frame->format = INPUT_FORMAT;
  input_frame->channel_layout = INPUT_CHANNEL_LAYOUT;
  input_frame->channels =
      av_get_channel_layout_nb_channels(INPUT_CHANNEL_LAYOUT);
  int32_t result = av_frame_get_buffer(input_frame, 0);
  if (result < 0) {
    std::cerr << "Error: AVFrame could not get buffer." << std::endl;
    return -1;
  }
  return 0;
}

int32_t audio_filtering() {
  int32_t result = 0;
  while (!end_of_input_file()) {
    result = init_frame();
    if (result < 0) {
      std::cerr << "Error: init_frame failed." << std::endl;
      return result;
    }
    result = read_pcm_to_frame2(input_frame, INPUT_FORMAT, 2);
    if (result < 0) {
      std::cerr << "Error: read_pcm_to_frame failed." << std::endl;
      return -1;
    }
    result = filter_frame();
    if (result < 0) {
      std::cerr << "Error: filter_frame failed." << std::endl;
      return -1;
    }
  }
  return result;
}

static void free_frames() {
  av_frame_free(&input_frame);
  av_frame_free(&output_frame);
}

void destroy_audio_filter() {
  free_frames();
  avfilter_graph_free(&filter_graph);
}
