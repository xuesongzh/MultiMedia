#ifndef AUDIO_ENCODER_CORE_H
#define AUDIO_ENCODER_CORE_H
#include <stdint.h>

int32_t init_audio_encoder(const char *codec_name);
int32_t audio_encoding();
void destroy_audio_encoder();

#endif