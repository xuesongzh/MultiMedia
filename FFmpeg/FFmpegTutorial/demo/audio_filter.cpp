#include <cstdlib>
#include <iostream>
#include <string>

#include "audio_filter_core.h"
#include "io_data.h"

static void usage(const char *program_name) {
  std::cout << "usage: " << std::string(program_name)
            << " input_file output_file " << std::endl;
}

int main(int argc, char **argv) {
  if (argc < 4) {
    usage(argv[0]);
    return -1;
  }

  char *input_file_name = argv[1];
  char *output_file_name = argv[2];
  char *volume_factor = argv[3];

  int32_t result = 0;
  do {
    result = open_input_output_files(input_file_name, output_file_name);
    if (result < 0) {
      break;
    }
    result = init_audio_filter(volume_factor);
    if (result < 0) {
      break;
    }
    result = audio_filtering();
    if (result < 0) {
      break;
    }
  } while (0);

  destroy_audio_filter();
  close_input_output_files();
  return result;
}
