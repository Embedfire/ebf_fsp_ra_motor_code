#include <motor_controls/bsp_motor_control.h>

/* 电机初始化 */
void Motor_Control_Init(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* 初始化 GPT 模块 */
    err = R_GPT_Open(&x_step_pwm_ctrl, &x_step_pwm_cfg);
    assert(FSP_SUCCESS == err);
    err = R_GPT_Enable(&x_step_pwm_ctrl);
    assert(FSP_SUCCESS == err);

    /* 初始化 GPT 模块 */
    err = R_GPT_Open(&y_step_pwm_ctrl, &y_step_pwm_cfg);
    assert(FSP_SUCCESS == err);
    err = R_GPT_Enable(&y_step_pwm_ctrl);
    assert(FSP_SUCCESS == err);

    X_STEP_ENABLED; //设置 X 轴电机使能
    Y_STEP_ENABLED; //设置 Y 轴电机使能
    X_STEP_CW;  // 设置 X 轴方向为正转
    Y_STEP_CW;  // 设置 Y 轴方向为正转
}

/**
 * @brief  启动电机
 */
void X_Motor_Control_Start(void)
{
    R_GPT_Start(&x_step_pwm_ctrl); // 使能定时器的比较输出
}

/**
 * @brief  启动电机
 */
void Y_Motor_Control_Start(void)
{
    R_GPT_Start(&y_step_pwm_ctrl); // 使能定时器的比较输出
}

/**
 * @brief  停止电机
 */
void X_Motor_Control_Stop(void)
{
    R_GPT_Stop(&x_step_pwm_ctrl);  // 禁用定时器的比较输出
}

/**
 * @brief  停止电机
 */
void Y_Motor_Control_Stop(void)
{
    R_GPT_Stop(&y_step_pwm_ctrl);  // 禁用定时器的比较输出
}


/**
 * @brief  控制电机速度
 * @param[in] speed_hz 目标速度，单位：Hz
 * @retval 无
 */
void Motor_Control_SetSpeed(uint32_t x_speed_hz, uint32_t y_speed_hz)
{
    // 使用 R_GPT_CompareMatchSet 和 R_GPT_PeriodSet 来设置定时器周期，从而控制 X 轴输出频率
    R_GPT_CompareMatchSet(&x_step_pwm_ctrl, x_speed_hz * 2, TIMER_COMPARE_MATCH_B);
    R_GPT_PeriodSet(&x_step_pwm_ctrl, Hz_Set(x_speed_hz));

    // 使用 R_GPT_CompareMatchSet 和 R_GPT_PeriodSet 来设置定时器周期，从而控制 Y 轴输出频率
    R_GPT_CompareMatchSet(&y_step_pwm_ctrl, y_speed_hz * 2, TIMER_COMPARE_MATCH_B);
    R_GPT_PeriodSet(&y_step_pwm_ctrl, Hz_Set(y_speed_hz));
}


