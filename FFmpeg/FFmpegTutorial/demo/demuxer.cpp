#include <cstdlib>
#include <iostream>
#include <string>

#include "demuxer_core.h"

static void usage(const char *program_name) {
  std::cout << "usage: " << std::string(program_name)
            << " input_file output_video_file output_audio_file " << std::endl;
}

int main(int argc, char **argv) {
  if (argc < 4) {
    usage(argv[0]);
    return 1;
  }
  do {
    int32_t result = init_demuxer(argv[1], argv[2], argv[3]);
    if (result < 0) {
      break;
    }
    result = demuxing(argv[2], argv[3]);
  } while (0);

end:
  destroy_demuxer();
  return 0;
}
