#ifndef AUDIO_DECODER_CORE_H
#define AUDIO_DECODER_CORE_H
#include <stdint.h>

int32_t init_audio_decoder(char *audio_codec_id);
void destroy_audio_decoder();
int32_t audio_decoding();

#endif