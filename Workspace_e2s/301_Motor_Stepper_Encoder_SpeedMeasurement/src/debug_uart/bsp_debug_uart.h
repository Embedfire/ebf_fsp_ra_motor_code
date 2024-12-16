#ifndef __BSP_DEBUG_UART_H
#define	__BSP_DEBUG_UART_H
#include "hal_data.h"
#include "stdio.h"

#define Clear_Buff()   memset( Order_Buff , 0 , sizeof(Order_Buff) ); 


void Debug_UART9_Init(void);

#define MOTOR_PRINT(fn_, ...)   printf (fn_, ##__VA_ARGS__);

#endif
