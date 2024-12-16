#include <motor_control/bsp_motor_control.h>
#include <adc/motor_v_c_acquisition.h>
#include "hal_data.h"
#include "pid/bsp_pid.h"
#include "gpt/bsp_gpt_pwm_output.h"
#include "protocol/protocol.h"

int8_t motor_pwm_duty = 50;        // 电机PWM占空比，初始值为10。
_Bool motor_dir;                   // 电机方向，0表示正向，1表示反向。
_Bool motor_state = false;
extern uint32_t filtered_current;                 // 滤波后的电流值

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
void Motor_Control_SetDirAndDuty(uint8_t dir, float pwm_count)
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
  * @brief  电机位置式 PID 控制实现(定时调用)
  * @param  无
  * @retval 无
  */
void motor_pid_control(float actual_current)
{

    uint32_t current = (uint32_t)actual_current;

    if(motor_state == true)

    {
    float cont_val = 0;                       // 当前控制值

    cont_val = PID_realize(actual_current);    // 进行 PID 计算

    if (cont_val < 0)
    {
      cont_val = 0;    // 下限处理
    }
    else if (cont_val > PWM_MAX_PERIOD_COUNT)
    {
      cont_val = PWM_MAX_PERIOD_COUNT;    // 速度上限处理
    }

    Motor_Control_SetDirAndDuty(motor_dir,cont_val);                                        // 设置 PWM 占空比

    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &current, 1);                // 给通道 1 发送实际值
    }

}


// GPT中断回调函数
void gpt0_timing_callback(timer_callback_args_t *p_args)
{
    // 定时器周期结束事件
    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        // 计算实际电流值
        float current = GET_ADC_CURR_VAL(GET_ADC_VDC_VAL((float)(filtered_current)));

        // 执行PID电机控制
        motor_pid_control(current);
    }
}


