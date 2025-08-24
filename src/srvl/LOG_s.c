#include "LOG_s.h"

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#define LOG_ERROR_LEVEL 1
#define LOG_WARN_LEVEL 2
#define LOG_INFO_LEVEL 3

void _INFO(const char* format, ...)
{
#if (LOG_DEBUG >= LOG_INFO_LEVEL)
    printf(ANSI_COLOR_BLUE "[INFO] ");

    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    printf(ANSI_COLOR_RESET "\r\n");
#endif
}

void _WARN(const char* format, ...)
{
#if (LOG_DEBUG >= LOG_WARN_LEVEL)
    printf(ANSI_COLOR_YELLOW "[WARN] ");

    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    printf(ANSI_COLOR_RESET "\r\n");
#endif
}

void _ERROR(const char* format, ...)
{
#if (LOG_DEBUG >= LOG_ERROR_LEVEL)
    printf(ANSI_COLOR_RED "[ERROR] ");

    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    printf(ANSI_COLOR_RESET "\r\n");
#endif
}
