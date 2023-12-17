#include <cstdlib>
#include <iostream>
#include <string>

#include "muxer_core.h"

static void usage(const char *program_name) {
  std::cout << "usage: " << std::string(program_name)
            << " video_file audio_file output_file " << std::endl;
}

int main(int argc, char **argv) {
  if (argc < 4) {
    usage(argv[0]);
    return 1;
  }
  int32_t result = 0;
  do {
    result = init_muxer(argv[1], argv[2], argv[3]);
    if (result < 0) {
      break;
    }
    result = muxing();
    if (result < 0) {
      break;
    }

  } while (0);
  destroy_muxer();

  return result;
}
