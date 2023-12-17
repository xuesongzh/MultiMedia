#include <cstdlib>
#include <iostream>
#include <string>

#include "io_data.h"
#include "video_decoder_core.h"

static void usage(const char *program_name) {
  std::cout << "usage: " << std::string(program_name)
            << " input_file output_file" << std::endl;
}

int main(int argc, char **argv) {
  if (argc < 3) {
    usage(argv[0]);
    return 1;
  }

  char *input_file_name = argv[1];
  char *output_file_name = argv[2];

  std::cout << "Input file:" << std::string(input_file_name) << std::endl;
  std::cout << "output file:" << std::string(output_file_name) << std::endl;

  int32_t result = open_input_output_files(input_file_name, output_file_name);
  if (result < 0) {
    return result;
  }

  result = init_video_decoder();
  if (result < 0) {
    return result;
  }

  result = decoding();
  if (result < 0) {
    return result;
  }

  destroy_video_decoder();
  close_input_output_files();
  return 0;
}
