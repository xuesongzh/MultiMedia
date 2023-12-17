#ifndef AUDIO_RESAMPLER_CORE_H
#define AUDIO_RESAMPLER_CORE_H
#include <stdint.h>

int32_t init_audio_resampler(int32_t in_sample_rate, const char *in_sample_fmt,
                             const char *in_ch_layout, int32_t out_sample_rate,
                             const char *out_sample_fmt,
                             const char *out_ch_layout);
int32_t audio_resampling();
void destroy_audio_resampler();

#endif