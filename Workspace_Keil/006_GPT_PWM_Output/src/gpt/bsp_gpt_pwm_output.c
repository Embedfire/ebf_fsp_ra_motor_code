#include "bsp_gpt_pwm_output.h"



void GPT_PWM_Init(void)
{
    /* 初始化 GPT 模块 */
    R_GPT_Open(&g_pwm4_ctrl, &g_pwm4_cfg);

    /* 启动 GPT 定时器 */
    R_GPT_Start(&g_pwm4_ctrl);

    /* 重新设置占空比为 50% */
    GPT_PWM_SetDuty(50);
}


/** 自定义函数：设置PWM占空比
    @param duty 占空比范围：0~100 %
*/
void GPT_PWM_SetDuty(uint8_t duty)
{
    timer_info_t info;
    uint32_t current_period_counts;
    uint32_t duty_cycle_counts;

    if (duty > 100)
        duty = 100; //限制占空比范围：0~100

    /* 获得GPT的信息 */
    R_GPT_InfoGet(&g_pwm4_ctrl, &info);

    /* 获得计时器一个周期需要的计数次数 */
    current_period_counts = info.period_counts;

    /* 根据占空比和一个周期的计数次数计算GTCCR寄存器的值 */
    duty_cycle_counts = (uint32_t)(((uint64_t) current_period_counts * duty) / 100);

    /* 最后调用FSP库函数设置占空比 */
    R_GPT_DutyCycleSet(&g_pwm4_ctrl, duty_cycle_counts, GPT_IO_PIN_GTIOCA_AND_GTIOCB);
}
