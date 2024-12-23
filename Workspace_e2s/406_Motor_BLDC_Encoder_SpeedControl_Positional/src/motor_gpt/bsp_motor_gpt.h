#ifndef R_BSP_MOTOR_GPT_H
#define R_BSP_MOTOR_GPT_H


#include "hal_data.h"

void hall_enable(void);
void hall_disable(void);
void hall_gpio_init(void);
void set_pwm_pulse(uint16_t pulse);
void update_speed_dir(uint8_t dir_in);
void update_motor_speed(uint32_t time);

void GPT1_Init(void);
void motor_gpt_init(void);
void stop_pwm_output(void);
void GPT_PWM_SetDuty(timer_ctrl_t * const p_ctrl,uint16_t duty,uint32_t const pin);

/* 电机控旋转实现结构体 */

#define SPEED_FILTER_NUM      30    // 速度滤波次数

typedef struct
{
  int32_t timeout;            // 定时器更新计数
  float speed;                // 电机速度 rps（转/分钟）
  float speed1;                // 电机速度 rps（转/分钟）
  int32_t enable_flag;        // 电机使能标志
  int32_t speed_group[SPEED_FILTER_NUM];
}motor_rotate_t;



#endif
