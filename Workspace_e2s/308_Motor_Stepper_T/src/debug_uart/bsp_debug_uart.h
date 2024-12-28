#ifndef __BSP_DEBUG_UART_H
#define __BSP_DEBUG_UART_H
#include "hal_data.h"
 #include "stdio.h"

 #include <sys/stat.h>
 #include <unistd.h>
 #include <errno.h>


void Debug_UART9_Init(void);
void ShowHelp(void);
void ShowData(int position, int accel_val, int decel_val, int speed, int steps);


#endif
