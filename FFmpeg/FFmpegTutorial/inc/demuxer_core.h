#ifndef DEMXUER_CORE_H
#define DEMXUER_CORE_H
#include <stdint.h>

int32_t init_demuxer(char *input_name, char *video_output, char *audio_output);
int32_t demuxing(char *video_output_name, char *audio_output_name);
void destroy_demuxer();
#endif