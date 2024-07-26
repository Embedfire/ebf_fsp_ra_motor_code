#include "bsp_gpt_pwm_output.h"


/*  当前选用PWM输出引脚 GTIOC6B P600 */


//仅用于计算占空比
static uint32_t current_period_counts1;
static uint32_t current_period_counts2;


/** GPT初始化函数
*/
void Motor_GPT_PWM_Init(void)
{
    timer_info_t info;

    /* 初始化 GPT 模块 */
    R_GPT_Open(&g_timer4_pwm_ctrl, &g_timer4_pwm_cfg);
    R_GPT_Open(&g_timer5_pwm_ctrl, &g_timer5_pwm_cfg);

    /* 获得计时器一个周期需要的计数次数 */
    R_GPT_InfoGet(&g_timer4_pwm_ctrl, &info);
    current_period_counts1 = info.period_counts;
    R_GPT_InfoGet(&g_timer5_pwm_ctrl, &info);
    current_period_counts2 = info.period_counts;

    R_GPT_Enable(&g_timer4_pwm_ctrl);
    R_GPT_Enable(&g_timer4_pwm_ctrl);

    /* 启动 GPT 定时器 */
    R_GPT_Start(&g_timer4_pwm_ctrl);
    R_GPT_Start(&g_timer5_pwm_ctrl);

    /* 初始化占空比为 0 */
    Motor_GPT_PWM_SetDuty(0, 0);
}


/** 设置PWM占空比函数

    @param duty_pwm1: 占空比范围：0~100 %
    @param duty_pwm2: 占空比范围：0~100 %
*/
void Motor_GPT_PWM_SetDuty(uint8_t duty_pwm1, uint8_t duty_pwm2)
{
    uint32_t duty_cycle_counts1;
    uint32_t duty_cycle_counts2;

    if (duty_pwm1 > 100)
        duty_pwm1 = 100; //限制占空比范围：0~100
    if (duty_pwm2 > 100)
        duty_pwm2 = 100; //限制占空比范围：0~100


    /* 根据占空比和一个周期的计数次数计算GTCCR寄存器的值 */
    /*执行 (current_period_counts1 - 100) 是为了防止H桥电路中的自举电容无法充电
     *从而导致高端MOSFET无法维持导通，电机缓慢停转*/
    duty_cycle_counts1 = (uint32_t)(((uint64_t) (current_period_counts1 - 100) * duty_pwm1) / 100);
    duty_cycle_counts2 = (uint32_t)(((uint64_t) (current_period_counts2 - 100) * duty_pwm2) / 100);

    /* 最后调用FSP库函数设置占空比 */
    R_GPT_DutyCycleSet(&g_timer4_pwm_ctrl, duty_cycle_counts1, GPT_IO_PIN_GTIOCA);
    R_GPT_DutyCycleSet(&g_timer5_pwm_ctrl, duty_cycle_counts2, GPT_IO_PIN_GTIOCA);
}
