#include "motor_gpt/bsp_motor_gpt.h"

/**
  * @brief  电机PWM定时器初始化
  * @param  无
  * @retval 无
  */
void motor_gpt_init(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* 初始化PWM定时器 */
    err = R_GPT_Open(&motor_u_ctrl, &motor_u_cfg);
    assert(FSP_SUCCESS == err);

    err = R_GPT_Open(&motor_v_ctrl, &motor_v_cfg);
    assert(FSP_SUCCESS == err);

    err = R_GPT_Open(&motor_w_ctrl, &motor_w_cfg);
    assert(FSP_SUCCESS == err);

    /* 启动 PWM1 定时器 */
    err = R_GPT_Start(&motor_u_ctrl);
    assert(FSP_SUCCESS == err);

    err = R_GPT_Start(&motor_v_ctrl);
    assert(FSP_SUCCESS == err);

    err = R_GPT_Start(&motor_w_ctrl);
    assert(FSP_SUCCESS == err);

}

/**
  * @brief  堵转超时定时器初始化
  * @param  无
  * @retval 无
  */
void Gpt0_Init(void)
{
    /* 初始化 GPT0 模块 */
    R_GPT_Open(&g_timer0_ctrl, &g_timer0_cfg);
}

/**
  * @brief  堵转超时定时器打开
  * @param  无
  * @retval 无
  */
void Gpt0_Open(void)
{
    /* 复位 GPT0 定时器 */
    R_GPT_Reset(&g_timer0_ctrl);
    /* 启动 GPT0 定时器 */
    R_GPT_Start(&g_timer0_ctrl);
}

/**
  * @brief  堵转超时定时器关闭
  * @param  无
  * @retval 无
  */
void Gpt0_Close(void)
{
    /* 关闭 GPT0 定时器 */
    R_GPT_Stop(&g_timer0_ctrl);
}

/**
  * @brief  停止电机PWM输出
  * @param  无
  * @retval 无
  */
void stop_pwm_output(void)
{
    GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
    GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
    GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);

    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);
}


/**
  * @brief  定时器占空比设置函数
  * @param  定时器、占空比、定时器引脚
  * @retval 无
  */
void GPT_PWM_SetDuty(timer_ctrl_t * const p_ctrl,uint16_t duty,uint32_t const pin)
{
    timer_info_t info;
    uint32_t current_period_counts;
    uint32_t duty_cycle_counts;

    if (duty > 100)
        duty = 100; //限制占空比范围：0~100

    /* 获得GPT的信息 */
    R_GPT_InfoGet(p_ctrl, &info);

    /* 获得计时器一个周期需要的计数次数 */
    current_period_counts = info.period_counts;

    /* 根据占空比和一个周期的计数次数计算GTCCR寄存器的值 */
    duty_cycle_counts = (uint32_t)(((uint64_t) current_period_counts * duty) / 100);

    /* 最后调用FSP库函数设置占空比 */
    R_GPT_DutyCycleSet(p_ctrl, duty_cycle_counts, pin);
}




