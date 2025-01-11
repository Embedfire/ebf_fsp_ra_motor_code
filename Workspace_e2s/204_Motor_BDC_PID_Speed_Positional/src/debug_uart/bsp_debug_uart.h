#ifndef __BSP_DEBUG_UART_H
#define __BSP_DEBUG_UART_H
#include "hal_data.h"
#include "stdio.h"

 #include <sys/stat.h>
 #include <unistd.h>
 #include <errno.h>

// 定义打印宏，允许格式化输出，类似 printf
#define MOTOR_PRINT(fn_, ...)   printf (fn_, ##__VA_ARGS__);


void Debug_UART9_Init(void);
void uart_protocol_send(uint8_t const * const p_src, uint32_t const bytes);

#endif
