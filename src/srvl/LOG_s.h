#ifndef _LOG_s_H_
#define _LOG_s_H_

#include <stdio.h>
#include <stdarg.h>

/**
  * @name   日志级别定义
  * @note   1：只打印错误日志
  * @note   2：打印错误和警告日志
  * @note   3：打印所有日志
  */
#define LOG_DEBUG 2

void _INFO(const char* format, ...) __attribute__((format(printf, 1, 2)));
void _WARN(const char* format, ...) __attribute__((format(printf, 1, 2)));
void _ERROR(const char* format, ...) __attribute__((format(printf, 1, 2)));

#endif
