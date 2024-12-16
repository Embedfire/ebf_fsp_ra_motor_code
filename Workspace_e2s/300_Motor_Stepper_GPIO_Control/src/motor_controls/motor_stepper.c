#include <motor_controls/motor_stepper.h>


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

//仅用于计算占空比
static uint32_t current_period_counts;



/**
 * @brief  设置电机速度
 *
 * 该函数通过修改PWM频率来控制电机的速度。
 *
 * @param[in] pwm_frequency  设置的PWM频率
 */
void Motor_Control_SetSpeed(uint8_t pwm_frequency)
{
    timer_info_t info;

    // 设置定时器的周期
    R_GPT_PeriodSet(&step_pwm_ctrl, KHz_Set((uint32_t)pwm_frequency));

    /* 获得计时器一个周期需要的计数次数 */
    R_GPT_InfoGet(&step_pwm_ctrl, &info);
    current_period_counts = info.period_counts;

    // 设置PWM占空比为50%
    R_GPT_DutyCycleSet(&step_pwm_ctrl, current_period_counts / 2, GPT_IO_PIN_GTIOCB);
}



