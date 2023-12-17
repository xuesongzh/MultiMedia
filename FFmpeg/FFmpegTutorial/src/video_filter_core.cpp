#include "video_filter_core.h"

#include <stdlib.h>
#include <string.h>

#include <iostream>

#include "io_data.h"
extern "C" {
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavutil/frame.h>
#include <libavutil/opt.h>
}

#define STREAM_FRAME_RATE 25

AVFilterContext *buffersink_ctx;
AVFilterContext *buffersrc_ctx;
AVFilterGraph *filter_graph;
AVFrame *input_frame = nullptr, *output_frame = nullptr;

static int32_t init_frames(int32_t width, int32_t height,
                           enum AVPixelFormat pix_fmt) {
  int result = 0;
  input_frame = av_frame_alloc();
  output_frame = av_frame_alloc();
  if (!input_frame || !output_frame) {
    std::cerr << "Error: frame allocation failed." << std::endl;
    return -1;
  }

  input_frame->width = width;
  input_frame->height = height;
  input_frame->format = pix_fmt;

  result = av_frame_get_buffer(input_frame, 0);
  if (result < 0) {
    std::cerr << "Error: could not get AVFrame buffer." << std::endl;
    return -1;
  }

  result = av_frame_make_writable(input_frame);
  if (result < 0) {
    std::cerr << "Error: input frame is not writable." << std::endl;
    return -1;
  }
  return 0;
}

int32_t init_video_filter(int32_t width, int32_t height,
                          const char *filter_descr) {
  int32_t result = 0;
  char args[512] = {0};
  const AVFilter *buffersrc = avfilter_get_by_name("buffer");
  const AVFilter *buffersink = avfilter_get_by_name("buffersink");
  AVFilterInOut *outputs = avfilter_inout_alloc();
  AVFilterInOut *inputs = avfilter_inout_alloc();
  AVRational time_base = (AVRational){1, STREAM_FRAME_RATE};
  enum AVPixelFormat pix_fmts[] = {AV_PIX_FMT_YUV420P, AV_PIX_FMT_NONE};

  do {
    filter_graph = avfilter_graph_alloc();
    if (!outputs || !inputs || !filter_graph) {
      std::cerr << "Error: creating filter graph failed." << std::endl;
      result = AVERROR(ENOMEM);
      break;
    }

    snprintf(args, sizeof(args),
             "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
             width, height, AV_PIX_FMT_YUV420P, 1, STREAM_FRAME_RATE, 1, 1);
    result = avfilter_graph_create_filter(&buffersrc_ctx, buffersrc, "in", args,
                                          NULL, filter_graph);
    if (result < 0) {
      std::cerr << "Error: could not create source filter." << std::endl;
      break;
    }

    result = avfilter_graph_create_filter(&buffersink_ctx, buffersink, "out",
                                          NULL, NULL, filter_graph);
    if (result < 0) {
      std::cerr << "Error: could not create sink filter." << std::endl;
      break;
    }

    result = av_opt_set_int_list(buffersink_ctx, "pix_fmts", pix_fmts,
                                 AV_PIX_FMT_NONE, AV_OPT_SEARCH_CHILDREN);
    if (result < 0) {
      std::cerr << "Error: could not set output pixel format." << std::endl;
      break;
    }

    outputs->name = av_strdup("in");
    outputs->filter_ctx = buffersrc_ctx;
    outputs->pad_idx = 0;
    outputs->next = NULL;

    inputs->name = av_strdup("out");
    inputs->filter_ctx = buffersink_ctx;
    inputs->pad_idx = 0;
    inputs->next = NULL;

    if ((result = avfilter_graph_parse_ptr(filter_graph, filter_descr, &inputs,
                                           &outputs, NULL)) < 0) {
      std::cerr << "Error: avfilter_graph_parse_ptr failed" << std::endl;
      break;
    }

    if ((result = avfilter_graph_config(filter_graph, NULL)) < 0) {
      std::cerr << "Error: Graph config invalid." << std::endl;
      break;
    }

    result = init_frames(width, height, AV_PIX_FMT_YUV420P);
    if (result < 0) {
      std::cerr << "Error: init frames failed." << std::endl;
      break;
    }
  } while (0);

  avfilter_inout_free(&inputs);
  avfilter_inout_free(&outputs);
  return result;
}

static int32_t filter_frame() {
  int32_t result = 0;
  if ((result = av_buffersrc_add_frame_flags(buffersrc_ctx, input_frame,
                                             AV_BUFFERSRC_FLAG_KEEP_REF)) < 0) {
    std::cerr << "Error: add frame to buffer src failed." << std::endl;
    return result;
  }

  while (1) {
    result = av_buffersink_get_frame(buffersink_ctx, output_frame);
    if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) {
      return 1;
    } else if (result < 0) {
      std::cerr << "Error: buffersink_get_frame failed." << std::endl;
      return result;
    }

    std::cout << "Frame filtered, width:" << output_frame->width
              << ", height : " << output_frame->height << std::endl;
    write_frame_to_yuv(output_frame);
    av_frame_unref(output_frame);
  }

  return result;
}

int32_t filtering_video(int32_t frame_cnt) {
  int32_t result = 0;
  for (size_t i = 0; i < frame_cnt; i++) {
    result = read_yuv_to_frame(input_frame);
    if (result < 0) {
      std::cerr << "Error: read_yuv_to_frame failed." << std::endl;
      return result;
    }

    result = filter_frame();
    if (result < 0) {
      std::cerr << "Error: filter_frame failed." << std::endl;
      return result;
    }
  }
  return result;
}

static void free_frames() {
  av_frame_free(&input_frame);
  av_frame_free(&output_frame);
}

void destroy_video_filter() {
  free_frames();
  avfilter_graph_free(&filter_graph);
}
