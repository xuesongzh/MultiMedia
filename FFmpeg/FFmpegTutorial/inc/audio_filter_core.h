#ifndef AUDIO_FILTER_CORE_H
#define AUDIO_FILTER_CORE_H
#include <stdint.h>

int32_t init_audio_filter(char* volume_factor);
int32_t audio_filtering();
void destroy_audio_filter();

#endif