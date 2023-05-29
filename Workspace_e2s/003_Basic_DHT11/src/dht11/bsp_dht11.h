#ifndef __BSP_DHT11_H
#define __BSP_DHT11_H
#include "hal_data.h"

#define DHT11_IO_OUTPUT  R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_13_PIN_13, IOPORT_CFG_PORT_DIRECTION_OUTPUT)
#define DHT11_IO_INPUT   R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_13_PIN_13, IOPORT_CFG_PORT_DIRECTION_INPUT)



void DHT11_RST(void);
void DHT11_IO_Init(void);


uint8_t DHT11_Check(void);
uint8_t DHT11_Read_Bit(void);
uint8_t DHT11_Read_Byte(void);
uint8_t DHT11_Init(void);
uint8_t DHT11_Read_Data(uint8_t *temp,uint8_t *humi);



#endif
