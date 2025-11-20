#include "s_LOG.h"

/* ========================= 私 有 变 量 声 明 ========================= */

#define ANSI_COLOR_RED      "\x1b[31m"
#define ANSI_COLOR_GREEN    "\x1b[32m"
#define ANSI_COLOR_YELLOW   "\x1b[33m"
#define ANSI_COLOR_BLUE     "\x1b[34m"
#define ANSI_COLOR_PURPLE   "\x1b[35m"
#define ANSI_COLOR_CYAN     "\x1b[36m"
#define ANSI_COLOR_RESET    "\x1b[0m"

#define LOG_ERROR_LEVEL 1
#define LOG_WARN_LEVEL 2
#define LOG_INFO_LEVEL 3

/* ========================= 接 口 函 数 实 现 ========================= */

/**
 * @brief 波形打印函数
 * @param count 变量个数
 * @param ... 可变参数，传入变量的指针
 * @note 变量类型必须是浮点数，如果非浮点数则传入&((float){value})的形式
 */
void _WavePrintf(int count, ...)
{
    printf(ANSI_COLOR_PURPLE "[WAVE] ");

    va_list args;
    va_start(args, count);

    for(int i = 0; i < count; ++i){
        float* ptr = (float*)va_arg(args, void*);
        if(i > 0) printf(",");
        printf("%f", *ptr);
    }
    va_end(args);

    printf(ANSI_COLOR_RESET "\r\n");
}

/**
 * @brief 信息级日志输出
 * @param format 格式化字符串
 * @param ... 可变参数
 */
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

/**
 * @brief 警告级日志输出
 * @param format 格式化字符串
 * @param ... 可变参数
 */
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

/**
 * @brief 错误级日志输出
 * @param format 格式化字符串
 * @param ... 可变参数
 */
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
