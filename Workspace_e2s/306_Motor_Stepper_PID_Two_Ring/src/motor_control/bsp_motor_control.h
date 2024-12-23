#ifndef BSP_MOTOR_CONTROL_H
#define BSP_MOTOR_CONTROL_H

#include "hal_data.h"
#include "encoder/bsp_encoder.h"

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

/*电机单圈参数*/
#define STEP_ANGLE        1.8f                        //步进电机的步距角 单位：度
#define FSPR              ((float)(360.0f/STEP_ANGLE))//步进电机的一圈所需脉冲数


#define MICRO_STEP        32                                //细分器细分数
#define SPR               (FSPR*MICRO_STEP)                 //细分后一圈所需脉冲数

#define TARGET_SPR        (float)(ENCODER_TOTAL_RESOLUTION/40)//定时器单位时间内（25ms）电机 1转/秒 应捕获的脉冲
#define PWM_MAX_FREQUENCY    19200  //步进电机所能承受的接近最大频率

#define SPEED_MAX_TARGET    TARGET_SPR*3  //步进电机速度最大目标

void Motor_Control_Init(void);
void Motor_Control_Start(void);
void Motor_Control_Stop(void);
void Motor_Control_Reverse(void);
void Motor_Control_SetSpeed(uint32_t pwm_frequency);
void motor_pid_control(float actual_location,float actual_speed);
#endif /* BSP_MOTOR_CONTROL_H */
