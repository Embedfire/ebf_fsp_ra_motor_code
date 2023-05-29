#ifndef __MOTOR_CONTROL_H_
#define __MOTOR_CONTROL_H_
#include "hal_data.h"

#define SD_HIGH_MOTOR_ENABLE    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_11_PIN_15, BSP_IO_LEVEL_HIGH)
#define SD_LOW_MOTOR_DISABLE    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_11_PIN_15, BSP_IO_LEVEL_LOW)


void Motor_Control_Init(void);
void Motor_Control_Start(void);
void Motor_Control_Stop(void);
void Motor_Control_Reverse(void);
void Motor_Control_SetDirAndDuty(uint8_t dir, uint8_t pwm_duty);

#endif /* MOTOR_CONTROL_MOTOR_CONTROL_H_ */
