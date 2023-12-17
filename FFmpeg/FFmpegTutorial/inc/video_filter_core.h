#ifndef VIDEO_FILTER_CORE_H
#define VIDEO_FILTER_CORE_H
#include <stdint.h>

int32_t init_video_filter(int32_t width, int32_t height,
                          const char *filter_descr);
int32_t filtering_video(int32_t frame_cnt);
void destroy_video_filter();
#endif