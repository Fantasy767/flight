#ifndef __COMMENT_H__
#define __COMMENT_H__

#include "usart.h"
#include "stdio.h"
#include "stdarg.h"
/*
//日志输出打印非常占用cpu资源，通过cpu计算的，在后续飞行时候，要关闭串口
//设计一个日志输出打印开关*/
#define DEBUG_LOG_ENABLE 1

#ifdef DEBUG_LOG_ENABLE
#define debug_printf(format, ...)  \
printf(format, ##__VA_ARGS__)

#else
#define debug_printf(format, ...)  
#endif

#endif