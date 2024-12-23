#ifndef R_BSP_MOTOR_GPT_H
#define R_BSP_MOTOR_GPT_H


#include "hal_data.h"

void hall_enable(void);
void hall_disable(void);
void hall_gpio_init(void);
void set_pwm_pulse(uint16_t pulse);
void update_motor_speed(uint32_t time);

void GPT1_Init(void);
void motor_gpt_init(void);
void stop_pwm_output(void);
void GPT_PWM_SetDuty(timer_ctrl_t * const p_ctrl,uint16_t duty,uint32_t const pin);



#endif
