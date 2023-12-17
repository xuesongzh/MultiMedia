extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
}

#include <cstdlib>
#include <iostream>
#include <string>

static std::string type_string(int type) {
  switch (type) {
    case AVIO_ENTRY_DIRECTORY:
      return "<DIR>";
    case AVIO_ENTRY_FILE:
      return "<FILE>";
    case AVIO_ENTRY_BLOCK_DEVICE:
      return "<BLOCK DEVICE>";
    case AVIO_ENTRY_CHARACTER_DEVICE:
      return "<CHARACTER DEVICE>";
    case AVIO_ENTRY_NAMED_PIPE:
      return "<PIPE>";
    case AVIO_ENTRY_SYMBOLIC_LINK:
      return "<LINK>";
    case AVIO_ENTRY_SOCKET:
      return "<SOCKET>";
    case AVIO_ENTRY_SERVER:
      return "<SERVER>";
    case AVIO_ENTRY_SHARE:
      return "<SHARE>";
    case AVIO_ENTRY_WORKGROUP:
      return "<WORKGROUP>";
    case AVIO_ENTRY_UNKNOWN:
    default:
      break;
  }
  return "<UNKNOWN>";
}

static int list_op(const char *input_dir) {
  AVIODirEntry *entry = NULL;
  AVIODirContext *ctx = NULL;
  int cnt, ret;
  char filemode[4], uid_and_gid[20];

  if ((ret = avio_open_dir(&ctx, input_dir, NULL)) < 0) {
    av_log(NULL, AV_LOG_ERROR, "Cannot open directory: %s.\n", av_err2str(ret));
    goto fail;
  }

  cnt = 0;
  for (;;) {
    if ((ret = avio_read_dir(ctx, &entry)) < 0) {
      av_log(NULL, AV_LOG_ERROR, "Cannot list directory: %s.\n",
             av_err2str(ret));
      goto fail;
    }
    if (!entry) break;
    if (entry->filemode == -1) {
      snprintf(filemode, 4, "???");
    } else {
      snprintf(filemode, 4, "%3" PRIo64, entry->filemode);
    }
    snprintf(uid_and_gid, 20, "%" PRId64 "(%" PRId64 ")", entry->user_id,
             entry->group_id);
    if (cnt == 0)
      av_log(NULL, AV_LOG_INFO, "%-9s %12s %30s %10s %s %16s %16s %16s\n",
             "TYPE", "SIZE", "NAME", "UID(GID)", "UGO", "MODIFIED", "ACCESSED",
             "STATUS_CHANGED");
    av_log(NULL, AV_LOG_INFO,
           "%-9s %12" PRId64 " %30s %10s %s %16" PRId64 " %16" PRId64
           " %16" PRId64 "\n",
           type_string(entry->type).c_str(), entry->size, entry->name,
           uid_and_gid, filemode, entry->modification_timestamp,
           entry->access_timestamp, entry->status_change_timestamp);
    avio_free_directory_entry(&entry);
    cnt++;
  };

fail:
  avio_close_dir(&ctx);
  return ret;
}

static void usage(const char *program_name) {
  std::cout << "usage: " << std::string(program_name) << " input_dir"
            << std::endl;
  std::cout << "API example program to show how to list files in directory "
               "accessed through AVIOContext."
            << std::endl;
}

int main(int argc, char *argv[]) {
  int ret;
  av_log_set_level(AV_LOG_DEBUG);
  if (argc < 2) {
    usage(argv[0]);
    return 1;
  }

  avformat_network_init();
  ret = list_op(argv[1]);
  avformat_network_deinit();

  return ret < 0 ? EXIT_FAILURE : EXIT_SUCCESS;
}