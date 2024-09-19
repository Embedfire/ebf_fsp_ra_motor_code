#ifndef MOTOR_STEPPER_H
#define MOTOR_STEPPER_H

#include "hal_data.h"


/*步进电机使能*/
#define  STEP_ENBLED	 R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_13_PIN_12, BSP_IO_LEVEL_LOW)
#define  STEP_DISENBLED	 R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_13_PIN_12, BSP_IO_LEVEL_HIGH)

/*步进电机方向控制*/
#define  STEP_CW	 	 R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_10, BSP_IO_LEVEL_HIGH)
#define  STEP_CCW	 	 R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_10, BSP_IO_LEVEL_LOW)

/*步进电机状态方向反转*/
#define  STEP_STATE_TOGGLE	 	 R_PORT13->PODR ^= 1<<(BSP_IO_PORT_13_PIN_12 & 0xFF)
#define  STEP_DIRECTION_TOGGLE	 R_PORT14->PODR ^= 1<<(BSP_IO_PORT_14_PIN_10 & 0xFF)

void initMotor(void);
void setStepSpeed(uint32_t speed);

#endif /* MOTOR_STEPPER_H */
