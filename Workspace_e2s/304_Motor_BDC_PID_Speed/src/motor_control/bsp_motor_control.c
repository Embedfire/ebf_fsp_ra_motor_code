#include <motor_control/bsp_motor_control.h>
#include "hal_data.h"

#include "gpt/bsp_gpt_pwm_output.h"


int8_t motor_pwm_duty = 10;        // 电机PWM占空比，初始值为42。
_Bool motor_dir;                   // 电机方向，0表示正向，1表示反向。

/*电机初始化*/
void Motor_Control_Init(void)
{
    /* 设置电机方向 */
    motor_dir = true; //正转
    Motor_Control_SetDirAndDuty(motor_dir, (uint8_t)motor_pwm_duty);
}

/**
 * @brief  启动电机
 */
void Motor_Control_Start(void)
{
    SD_HIGH_MOTOR_ENABLE; // 使能电机

    R_GPT_Start(&g_timer4_pwm_ctrl);
    R_GPT_Start(&g_timer5_pwm_ctrl);
}


/**
 * @brief  停止电机
 */
void Motor_Control_Stop(void)
{
    SD_LOW_MOTOR_DISABLE; // 禁用电机
    R_GPT_Stop(&g_timer4_pwm_ctrl);
    R_GPT_Stop(&g_timer5_pwm_ctrl);
}

/**
 * @brief  反转电机方向
 */
void Motor_Control_Reverse(void)
{
    //生成一个软件事件来触发 ‘清除计数’ 操作
    R_ELC_SoftwareEventGenerate(&g_elc_ctrl, ELC_SOFTWARE_EVENT_0);

    // 反转电机方向
    motor_dir = !motor_dir;

    // 根据新的电机方向设置 PWM 占空比
    if (motor_dir == false) // 顺时针方向
    {
        Motor_GPT_PWM_SetDuty((uint8_t)motor_pwm_duty, 0); // 设置正向占空比
    }
    else // 逆时针方向
    {
        Motor_GPT_PWM_SetDuty(0, (uint8_t)motor_pwm_duty); // 设置反向占空比
    }
}


/**
 * @brief  设置电机方向和占空比
 * @param  dir 方向 (0: 正向, 1: 反向)
 * @param  pwm_duty 占空比 (0-100)
 */
void Motor_Control_SetDirAndDuty(uint8_t dir, float pwm_duty)
{
    if (false == dir) // 如果方向为0
        Motor_GPT_PWM_SetDuty(pwm_duty, 0); // 设置正向占空比
    else
        Motor_GPT_PWM_SetDuty(0, pwm_duty); // 设置反向占空比
}

