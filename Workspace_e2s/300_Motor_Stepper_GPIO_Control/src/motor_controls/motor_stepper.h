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


#define TIMER_CLK_FREQ 120000000  // 120 MHz
// 计算目标频率（单位：kHz）对应的定时器周期值
#define KHz_Set(frequency_khz)  ((TIMER_CLK_FREQ / ((frequency_khz) * 1000)) - 1)

// 计算目标频率（单位：Hz）对应的定时器周期值
#define Hz_Set(frequency_hz)  ((TIMER_CLK_FREQ / (frequency_hz)) - 1)


void Motor_Control_Init(void);
void Motor_Control_Start(void);
void Motor_Control_Stop(void);
void Motor_Control_Reverse(void);
void Motor_Control_SetSpeed(uint8_t pwm_frequency);

#endif /* MOTOR_STEPPER_H */
