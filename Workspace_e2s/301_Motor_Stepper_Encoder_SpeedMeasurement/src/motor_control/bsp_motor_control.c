#include "bsp_motor_control.h"


_Bool motor_dir = true;                   // 电机方向，0表示正向，1表示反向。

/*电机初始化*/
void Motor_Control_Init(void)
{

    fsp_err_t err = FSP_SUCCESS;
    /* 初始化 GPT 模块 */
    err = R_GPT_Open(&step_pwm_ctrl, &step_pwm_cfg);
    assert(FSP_SUCCESS == err);
    
    /* 设置电机方向 */
    STEP_CCW;
}

/**
 * @brief  启动电机
 */
void Motor_Control_Start(void)
{
    STEP_ENBLED; // 使能电机
    R_GPT_Start(&step_pwm_ctrl);// 使能定时器7的PWM输出

}

/**
 * @brief  停止电机
 */
void Motor_Control_Stop(void)
{
    STEP_DISENBLED;              // 禁用电机
    R_GPT_Stop(&step_pwm_ctrl);  // 禁用定时器7的PWM输出
}

/**
 * @brief  反转电机方向
 */
void Motor_Control_Reverse(void)
{
    motor_dir = !motor_dir;      // 切换电机方向
    STEP_DIRECTION_TOGGLE;      // 切换电机方向控制引脚
}

/**
 * @brief  控制电机速度
 * @param[in] speed_hz 目标速度，单位：Hz
 * @retval 无
 */
void Motor_Control_SetSpeed(uint32_t speed_hz)
{
    // 使用 R_GPT_PeriodSet 来设置定时器周期，从而控制 PWM 输出频率
    R_GPT_PeriodSet(&step_pwm_ctrl, Hz_Set(speed_hz));
}




