#ifndef __BSP_GPT_PWM_OUTPUT_H
#define __BSP_GPT_PWM_OUTPUT_H
#include "hal_data.h"


void Motor_GPT_PWM_Init(void);
void Motor_GPT_PWM_SetDuty(float duty_pwm1, float duty_pwm2);

#endif
