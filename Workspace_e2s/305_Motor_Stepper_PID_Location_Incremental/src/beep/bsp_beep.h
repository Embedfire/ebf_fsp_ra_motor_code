#ifndef __BSP_BEEP_H
#define __BSP_BEEP_H
#include "hal_data.h"


#define BUZZER_ON     R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_13_PIN_04, BSP_IO_LEVEL_HIGH)


#define BUZZER_OFF    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_13_PIN_04, BSP_IO_LEVEL_LOW)


void BEEP_Init(void);





#endif
