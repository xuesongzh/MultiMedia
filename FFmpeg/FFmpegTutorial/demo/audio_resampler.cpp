#include <cstdlib>
#include <iostream>
#include <string>

#include "audio_resampler_core.h"
#include "io_data.h"

static void usage(const char *program_name) {
  std::cout << "usage: " << std::string(program_name)
            << " in_file in_sample_rate in_sample_fmt out_file out_sample_fmt "
               "out_sample_fmt "
            << std::endl;
}

int main(int argc, char **argv) {
  int result = 0;
  if (argc < 7) {
    usage(argv[0]);
    return -1;
  }

  char *input_file_name = argv[1];
  int32_t in_sample_rate = atoi(argv[2]);
  char *in_sample_fmt = argv[3];
  char *in_sample_layout = argv[4];

  char *output_file_name = argv[5];
  int32_t out_sample_rate = atoi(argv[6]);
  char *out_sample_fmt = argv[7];
  char *out_sample_layout = argv[8];

  do {
    result = open_input_output_files(input_file_name, output_file_name);
    if (result < 0) {
      break;
    }
    result = init_audio_resampler(in_sample_rate, in_sample_fmt,
                                  in_sample_layout, out_sample_rate,
                                  out_sample_fmt, out_sample_layout);
    if (result < 0) {
      std::cerr << "Error: init_audio_resampler failed." << std::endl;
      return result;
    }
    result = audio_resampling();
    if (result < 0) {
      std::cerr << "Error: audio_resampling failed." << std::endl;
      return result;
    }
  } while (0);

  close_input_output_files();
  destroy_audio_resampler();
  return result;
}
