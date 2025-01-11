#ifndef __BSP_DEBUG_UART_H
#define __BSP_DEBUG_UART_H
#include "hal_data.h"
#include "stdio.h"

// 清空 Order 变量，设置为 null 字符
#define Clear_Order   Order = '\0';

// 定义打印宏，允许格式化输出，类似 printf
#define MOTOR_PRINT(fn_, ...)   printf (fn_, ##__VA_ARGS__);


void Debug_UART9_Init(void);


#endif
