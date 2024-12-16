#ifndef __BSP_DEBUG_UART_H
#define	__BSP_DEBUG_UART_H
#include "hal_data.h"
#include "stdio.h"


void Debug_UART9_Init(void);
void uart_protocol_send(uint8_t const * const p_src, uint32_t const bytes);


#endif
