#ifndef MOTOR_STEPPER_H
#define MOTOR_STEPPER_H

#include "hal_data.h"


/* 步进电机使能控制 */
#define STEP_ENABLED       R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_13_PIN_12, BSP_IO_LEVEL_LOW)   // 步进电机使能
#define STEP_DISABLED      R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_13_PIN_12, BSP_IO_LEVEL_HIGH)  // 步进电机禁用

/* 步进电机方向控制 */
#define STEP_CW            R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_10, BSP_IO_LEVEL_HIGH)  // 设置步进电机顺时针方向
#define STEP_CCW           R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_10, BSP_IO_LEVEL_LOW)   // 设置步进电机逆时针方向

/* 步进电机状态及方向反转 */
#define STEP_STATE_TOGGLE      R_PORT13->PODR ^= 1 << (BSP_IO_PORT_13_PIN_12 & 0xFF)  // 切换步进电机的使能状态
#define STEP_DIRECTION_TOGGLE  R_PORT14->PODR ^= 1 << (BSP_IO_PORT_14_PIN_10 & 0xFF)  // 切换步进电机的方向

/* 定时器时钟频率 */
#define TIMER_CLK_FREQ       120000000  // 定时器时钟频率为 120 MHz

/* 计算目标频率对应的定时器周期值 */
#define Hz_Set(frequency_hz) ((TIMER_CLK_FREQ / (frequency_hz)) - 1)  // 根据目标频率计算周期值

/* 电机单圈参数 */
#define STEP_ANGLE           1.8f                        // 步进电机步距角（单位：度）
#define FSPR                 (360.0f / STEP_ANGLE)       // 电机一圈所需步数（无细分）

#define MICRO_STEP           32                          // 细分数
#define SPR                  ((uint32_t)(FSPR * MICRO_STEP))  // 细分后电机一圈所需脉冲数
#define QUARTER_SPR          (SPR / 4)                   // 细分后四分之一圈所需脉冲数

/* 步进电机 PWM 参数 */
#define PWM_MAX_FREQUENCY    19200  // 步进电机最大频率
#define PWM_MIN_FREQUENCY    1600   // 步进电机最小频率


void Motor_Control_Init(void);
void Motor_Control_Start(void);
void Motor_Control_Stop(void);
void Motor_Control_Reverse(void);
void Motor_Control_SetSpeed(uint32_t speed_hz);

#endif /* MOTOR_STEPPER_H */
