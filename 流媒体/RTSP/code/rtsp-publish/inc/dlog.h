#ifndef DLOG_H
#define DLOG_H

#include <stdio.h>

#ifndef NULL
#define NULL (0)
#endif

#ifndef TRUE
#define TRUE (1)
#endif

#ifndef FALSE
#define FALSE (0)
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum _slog_level {
    S_TRACE = 1,
    S_DEBUG = 2,
    S_INFO = 3,
    S_WARN = 4,
    S_ERROR = 5
} slog_level;

int init_logger(const char *log_dir, slog_level level);
void write_log(slog_level level, int print_stacktrace, const char *func_name, int line, const char *fmt, ...);

#define LogError(fmt, ...) write_log(S_ERROR, FALSE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LogWarn(fmt, ...) write_log(S_WARN, FALSE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LogInfo(fmt, ...) write_log(S_INFO, FALSE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LogDebug(fmt, ...) write_log(S_DEBUG, FALSE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LogTrace(fmt, ...) write_log(S_TRACE, FALSE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif  // DLOG_H
