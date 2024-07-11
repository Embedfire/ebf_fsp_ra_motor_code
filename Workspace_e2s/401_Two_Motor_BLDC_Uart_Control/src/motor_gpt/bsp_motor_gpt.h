#ifndef R_BSP_MOTOR_GPT_H
#define R_BSP_MOTOR_GPT_H


#include "hal_data.h"

void Gpt0_Init(void);
void Gpt0_Open(void);
void Gpt0_Close(void);
void motor1_gpt_init(void);
void motor2_gpt_init(void);
void motor1_stop_pwm_output(void);
void motor2_stop_pwm_output(void);
void GPT_PWM_SetDuty(timer_ctrl_t * const p_ctrl,uint16_t duty,uint32_t const pin);



#endif
