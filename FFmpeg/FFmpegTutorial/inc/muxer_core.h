#ifndef MUXER_CORE_H
#define MUXER_CORE_H
#include <stdint.h>
int32_t init_muxer(char *video_input_file, char *audio_input_file,
                   char *output_file);
int32_t muxing();
void destroy_muxer();
#endif