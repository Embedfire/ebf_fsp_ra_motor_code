#include "motor_control/bsp_motor_control.h"
#include "adc/motor_v_c_acquisition.h"
#include "hal_data.h"
#include "pid/bsp_pid.h"
#include "encoder/bsp_encoder.h"
#include "gpt/bsp_gpt_pwm_output.h"
#include "protocol/protocol.h"


int8_t motor_pwm_duty = 50;        // 电机PWM占空比，初始值为50。
_Bool motor_dir;                   // 电机方向，0表示正向，1表示反向。
_Bool motor_state = false;
extern volatile uint32_t pulse_period;   // 脉冲数
extern volatile int32_t pulse_direction;   // 方向脉冲
extern uint32_t filtered_current;          // 滤波后的电流值

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
 * @brief  电机 PID 控制实现（定时调用）
 *
 * 通过三环控制（位置环、速度环、电流环），根据电机的实际位置、速度和电流，
 * 动态调整电机方向与 PWM 占空比，实现精准控制。
 *
 * @param[in] actual_location 当前电机实际位置
 * @param[in] actual_speed    当前电机实际速度
 * @param[in] actual_current  当前电机实际电流
 * @retval 无
 */
void motor_pid_control(float actual_location, float actual_speed, float actual_current)
{
    int32_t location = (int32_t)actual_location;   // 将实际位置转换为整数类型
    uint32_t speed = (uint32_t)actual_speed;       // 将实际速度转换为整数类型
    uint32_t current = (uint32_t)actual_current;   // 将实际电流转换为整数类型

    static uint32_t louter_ring_timer = 0;         // 外环周期计数器（控制不同环的执行周期）
    float cont_val = 0;                            // 当前控制值

    // 限制实际电流最大值，避免过载
    if (actual_current > TARGET_CURRENT_MAX)
    {
        actual_current = TARGET_CURRENT_MAX;
    }

    // 当电机处于启用状态时执行控制
    if (motor_state == true)
    {
        /* 位置环计算（每 3 个周期执行一次） */
        if (louter_ring_timer++ % 3 == 0)
        {
            // 通过位置 PID 计算目标速度
            cont_val = location_pid_realize(&pid_location, actual_location);

            // 根据计算结果调整电机方向
            if (cont_val > 0)
            {
                motor_dir = true;  // 正方向
            }
            else if (cont_val < 0)
            {
                cont_val = -cont_val;
                motor_dir = false; // 反方向
            }

            // 限制目标速度的最大值
            if (cont_val > TARGET_SPEED_MAX)
            {
                cont_val = TARGET_SPEED_MAX;
            }

            // 将目标速度传递给速度环
            set_pid_target(&pid_speed, cont_val);

            // 向上位机发送目标速度值（通道 2）
            uint32_t temp = (uint32_t)cont_val;
            set_computer_value(SEND_TARGET_CMD, CURVES_CH2, &temp, 1);
        }

        /* 速度环计算（每 2 个周期执行一次） */
        if (louter_ring_timer % 2 == 0)
        {
            // 通过速度 PID 计算目标电流
            cont_val = speed_pid_realize(&pid_speed, actual_speed);

            // 限制目标电流的最大值
            if (cont_val > TARGET_CURRENT_MAX)
            {
                cont_val = TARGET_CURRENT_MAX;
            }

            // 将目标电流传递给电流环
            set_pid_target(&pid_curr, cont_val);

            // 向上位机发送目标电流值（通道 3）
            uint32_t temp = (uint32_t)cont_val;
            set_computer_value(SEND_TARGET_CMD, CURVES_CH3, &temp, 1);
        }

        /* 电流环计算（每个周期执行一次） */
        // 通过电流 PID 计算最终的 PWM 占空比
        cont_val = curr_pid_realize(&pid_curr, actual_current);

        // 限制 PWM 占空比的上下限
        if (cont_val < 0)
        {
            cont_val = 0;  // 下限处理
        }
        else if (cont_val > PWM_MAX_PERIOD_COUNT)
        {
            cont_val = PWM_MAX_PERIOD_COUNT;  // 上限处理
        }

        // 设置电机方向和 PWM 占空比
        Motor_Control_SetDirAndCount(motor_dir, cont_val);

        // 向上位机发送当前实际值（通道 1、2、3 分别为位置、速度、电流）
        set_computer_value(SEND_FACT_CMD, CURVES_CH1, &location, 1); // 位置
        set_computer_value(SEND_FACT_CMD, CURVES_CH2, &speed, 1);    // 速度
        set_computer_value(SEND_FACT_CMD, CURVES_CH3, &current, 1);  // 电流
    }
}



/* GPT定时器中断回调函数 */
void gpt0_timing_callback(timer_callback_args_t *p_args)
{
    static uint32_t last_pulse_period = 0; // 上一次的脉冲周期
    static uint32_t new_period = 0; // 当前脉冲周期
    static float shaft_speed = 0.00f; // 转轴速度

    if (TIMER_EVENT_CYCLE_END == p_args->event)  // 检查定时器周期结束事件
    {
        // 计算实际电流值
        float current = GET_ADC_CURR_VAL(GET_ADC_VDC_VAL((float)(filtered_current)));

        // 更新最近的脉冲周期
        new_period = (pulse_period - last_pulse_period);

        // 计算转轴速度（单位：转/秒）
        shaft_speed = SHAFT_SPEED(new_period) * 40.0f;

        motor_pid_control((float)pulse_direction,shaft_speed,current);
    }
        // 更新脉冲周期
        last_pulse_period = pulse_period;
}

