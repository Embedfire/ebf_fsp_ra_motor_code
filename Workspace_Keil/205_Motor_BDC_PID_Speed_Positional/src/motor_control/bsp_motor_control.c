#include <motor_control/bsp_motor_control.h>
#include "hal_data.h"
#include "pid/bsp_pid.h"
#include "encoder/bsp_encoder.h"
#include "gpt/bsp_gpt_pwm_output.h"
#include "protocol/protocol.h"


int8_t motor_pwm_duty = 50;        // 电机PWM占空比，初始值为50。
_Bool motor_dir;                   // 电机方向，0表示正向，1表示反向。
_Bool motor_state = false;
extern volatile uint32_t pulse_period;   // 脉冲数

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
    motor_state = true;
    SD_HIGH_MOTOR_ENABLE; // 使能电机
    R_GPT_OutputEnable(&g_timer4_pwm_ctrl, GPT_IO_PIN_GTIOCA); // 使能定时器4的PWM输出
    R_GPT_OutputEnable(&g_timer5_pwm_ctrl, GPT_IO_PIN_GTIOCA); // 使能定时器5的PWM输出
}

/**
 * @brief  停止电机
 */
void Motor_Control_Stop(void)
{
    motor_state = false;
    SD_LOW_MOTOR_DISABLE; // 禁用电机
    R_GPT_OutputDisable(&g_timer4_pwm_ctrl, GPT_IO_PIN_GTIOCA); // 禁用定时器4的PWM输出
    R_GPT_OutputDisable(&g_timer5_pwm_ctrl, GPT_IO_PIN_GTIOCA); // 禁用定时器5的PWM输出
}

/**
 * @brief  反转电机方向
 */
void Motor_Control_Reverse(void)
{
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
void Motor_Control_SetDirAndDuty(uint8_t dir, uint8_t pwm_duty)
{
    if (false == dir) // 如果方向为0
        Motor_GPT_PWM_SetDuty(pwm_duty, 0); // 设置正向占空比
    else
        Motor_GPT_PWM_SetDuty(0, pwm_duty); // 设置反向占空比
}

/**
 * @brief  设置电机方向和计数值
 * @param  dir 方向 (0: 正向, 1: 反向)
 * @param  pwm_duty 占空比 (0-100)
 */
void Motor_Control_SetDirAndCount(uint8_t dir, float pwm_count)
{
    if (false == dir) // 如果方向为0
    {
        R_GPT_DutyCycleSet(&g_timer4_pwm_ctrl, (uint32_t)pwm_count, GPT_IO_PIN_GTIOCA); // 设置正向占空比
        R_GPT_DutyCycleSet(&g_timer5_pwm_ctrl, 0, GPT_IO_PIN_GTIOCA);
    }
    else
    {
        R_GPT_DutyCycleSet(&g_timer4_pwm_ctrl, 0, GPT_IO_PIN_GTIOCA); // 设置反向占空比
        R_GPT_DutyCycleSet(&g_timer5_pwm_ctrl, (uint32_t)pwm_count, GPT_IO_PIN_GTIOCA);
    }
}


/**
 * @brief  电机位置式 PID 控制实现（定时调用）
 *
 * 根据当前电机位置与目标位置，通过 PID 控制算法计算并调整电机 PWM 占空比，进而控制电机运动。
 *
 * @param[in] actual_current  当前电机实际位置（如速度、位置等）
 * @retval 无
 */
void motor_pid_control(float actual_speed)
{
    uint32_t speed = (uint32_t)actual_speed;   // 将实际位置转换为整数类型

    if (motor_state == true)   // 当电机处于启用状态时
    {
        float cont_val = 0;   // 当前控制值

        // 进行 PID 计算
        cont_val = PID_realize(actual_speed);

        // 控制值下限处理
        if (cont_val < 0)
        {
            cont_val = 0;    // 下限为 0
        }
        // 控制值上限处理
        else if (cont_val > PWM_MAX_PERIOD_COUNT)
        {
            cont_val = PWM_MAX_PERIOD_COUNT;    // 上限为 PWM 最大周期
        }

        // 设置电机方向和 PWM 占空比
        Motor_Control_SetDirAndCount(motor_dir, cont_val);

        // 向通道 1 发送当前实际位置值
        set_computer_value(SEND_FACT_CMD, CURVES_CH1, &speed, 1);
    }
}


/* GPT定时器中断回调函数 */
void gpt0_timing_callback(timer_callback_args_t *p_args)
{
    static uint32_t last_pulse_period = 0; // 上一次的脉冲周期
    static uint32_t new_period = 0; // 上一次的脉冲周期
    static float shaft_speed = 0.00f;

    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        if (pulse_period > 0 && pulse_period != last_pulse_period)
        {
            // 更新最近速度
            new_period = (pulse_period - last_pulse_period);

            // 转轴速度计算（单位：转/秒）
            shaft_speed = SHAFT_SPEED(new_period) * 40.0f;

            motor_pid_control(shaft_speed);

       }
        last_pulse_period = pulse_period; // 更新目前脉冲周期
    }
}
