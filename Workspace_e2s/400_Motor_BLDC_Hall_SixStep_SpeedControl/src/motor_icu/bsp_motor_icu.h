#ifndef R_BSP_MOTOR_ICU_H
#define R_BSP_MOTOR_ICU_H


#include "hal_data.h"

void hall_enable(void);
void hall_disable(void);
void hall_gpio_init(void);
void set_pwm_pulse(uint16_t pulse);

#endif
