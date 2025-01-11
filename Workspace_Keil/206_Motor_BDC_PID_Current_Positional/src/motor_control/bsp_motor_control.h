#ifndef __MOTOR_CONTROL_H_
#define __MOTOR_CONTROL_H_
#include "hal_data.h"



// 启用电机，将引脚BSP_IO_PORT_11_PIN_15设置为高电平
#define SD_HIGH_MOTOR_ENABLE    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_11_PIN_15, BSP_IO_LEVEL_HIGH)
// 禁用电机，将引脚BSP_IO_PORT_11_PIN_15设置为低电平
#define SD_LOW_MOTOR_DISABLE    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_11_PIN_15, BSP_IO_LEVEL_LOW)



#define PWM_PERIOD_COUNT     (0x2dc6c0)

/* 最大比较值 */
#define PWM_MAX_PERIOD_COUNT              (PWM_PERIOD_COUNT - 100)


void Motor_Control_Init(void);
void Motor_Control_Start(void);
void Motor_Control_Stop(void);
void Motor_Control_Reverse(void);
void Motor_Control_SetDirAndDuty(uint8_t dir, float pwm_count);
void motor_pid_control(float actual_current);


#endif /* MOTOR_CONTROL_MOTOR_CONTROL_H_ */
