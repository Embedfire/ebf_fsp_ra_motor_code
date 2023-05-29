#ifndef R_BLDC_H
#define R_BLDC_H
#include "hal_data.h"

/*使能引脚*/
#define  MOTOR_ENBLE	 	 R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_11_PIN_15, BSP_IO_LEVEL_HIGH); 
#define  MOTOR_DISENBLE		 R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_11_PIN_15, BSP_IO_LEVEL_LOW); 

#define  MOTOR_STATE_TOGGLE 	 R_PORT11->PODR ^= 1<<(BSP_IO_PORT_11_PIN_15 & 0xFF)

void initMotor(void);
bsp_io_level_t Get_Key_State(bsp_io_port_pin_t pin);
uint8_t Get_Hall_State(void);
void GPT_PWM_SetDuty(timer_ctrl_t * const p_ctrl,uint8_t duty,uint32_t const pin);
void Change(void);

#endif 
