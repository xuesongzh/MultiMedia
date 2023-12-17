#include <cstdlib>
#include <iostream>
#include <string>

#include "io_data.h"
#include "video_filter_core.h"

static void usage(const char *program_name) {
  std::cout << "usage: " << std::string(program_name)
            << " input_file pic_width pic_height pix_fmt filter_discr"
            << std::endl;
}

int main(int argc, char **argv) {
  if (argc < 6) {
    usage(argv[0]);
    return 1;
  }
  char *input_file_name = argv[1];
  int32_t pic_width = atoi(argv[2]);
  int32_t pic_height = atoi(argv[3]);
  int32_t total_frame_cnt = atoi(argv[4]);
  char *filter_descr = argv[5];
  char *output_file_name = argv[6];

  int32_t result = open_input_output_files(input_file_name, output_file_name);
  if (result < 0) {
    return result;
  }

  result = init_video_filter(pic_width, pic_height, filter_descr);
  if (result < 0) {
    return result;
  }

  result = filtering_video(total_frame_cnt);
  if (result < 0) {
    return result;
  }

  close_input_output_files();
  destroy_video_filter();

  return 0;
}
