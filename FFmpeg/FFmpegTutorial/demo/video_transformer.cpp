#include <cstdlib>
#include <iostream>
#include <string>

#include "io_data.h"
#include "video_swscale_core.h"

static void usage(const char *program_name) {
  std::cout << "usage: " << std::string(program_name)
            << " input_file input_size in_pix_fmt in_layout output_file "
               "output_size out_pix_fmt out_layout "
            << std::endl;
}

int main(int argc, char **argv) {
  int result = 0;
  if (argc < 7) {
    usage(argv[0]);
    return -1;
  }

  char *input_file_name = argv[1];
  char *input_pic_size = argv[2];
  char *input_pix_fmt = argv[3];
  char *output_file_name = argv[4];
  char *output_pic_size = argv[5];
  char *output_pix_fmt = argv[6];

  do {
    result = open_input_output_files(input_file_name, output_file_name);
    if (result < 0) {
      break;
    }
    result = init_video_swscale(input_pic_size, input_pix_fmt, output_pic_size,
                                output_pix_fmt);
    if (result < 0) {
      break;
    }
    result = transforming(100);
    if (result < 0) {
      break;
    }
  } while (0);

failed:
  destroy_video_swscale();
  close_input_output_files();
  return result;
}
