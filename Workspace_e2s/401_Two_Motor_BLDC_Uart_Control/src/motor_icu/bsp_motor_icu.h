#ifndef R_BSP_MOTOR_ICU_H
#define R_BSP_MOTOR_ICU_H


#include "hal_data.h"

void motor1_hall_enable(void);
void motor2_hall_enable(void);
void motor1_hall_disable(void);
void motor2_hall_disable(void);
void motor1_hall_gpio_init(void);
void motor2_hall_gpio_init(void);
void motor1_set_pwm_pulse(uint16_t pulse);
void motor2_set_pwm_pulse(uint16_t pulse);

#endif
