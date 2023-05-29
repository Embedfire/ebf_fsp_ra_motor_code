#include "hal_data.h"

#include "motor_control.h"
#include "gpt/bsp_gpt_pwm_output.h"



int8_t motor_pwm_duty=50;
uint8_t motor_dir;
volatile int16_t encoder_counter_value;
volatile uint8_t encoder_counter_dir;


void Motor_Control_Init(void)
{
    /* 设置电机方向 */
    motor_dir = 0; //正转
    Motor_Control_SetDirAndDuty(motor_dir, (uint8_t)motor_pwm_duty);
}

void Motor_Control_Start(void)
{
    SD_HIGH_MOTOR_ENABLE;
    R_GPT_OutputEnable(&g_timer4_pwm_ctrl, GPT_IO_PIN_GTIOCA);
    R_GPT_OutputEnable(&g_timer5_pwm_ctrl, GPT_IO_PIN_GTIOCA);
}

void Motor_Control_Stop(void)
{
    SD_LOW_MOTOR_DISABLE;
    R_GPT_OutputDisable(&g_timer4_pwm_ctrl, GPT_IO_PIN_GTIOCA);
    R_GPT_OutputDisable(&g_timer5_pwm_ctrl, GPT_IO_PIN_GTIOCA);
}

void Motor_Control_Reverse(void)
{
    if (0 == motor_dir)
        motor_dir++;
    else
        motor_dir--;

    if (0 == motor_dir)
        Motor_GPT_PWM_SetDuty((uint8_t)motor_pwm_duty, 0);
    else
        Motor_GPT_PWM_SetDuty(0, (uint8_t)motor_pwm_duty);
}

void Motor_Control_SetDirAndDuty(uint8_t dir, uint8_t pwm_duty)
{
    if (0 == dir)
        Motor_GPT_PWM_SetDuty(pwm_duty, 0);
    else
        Motor_GPT_PWM_SetDuty(0, pwm_duty);
}

