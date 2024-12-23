#include "bsp_gpt_timing.h"
#include "led/bsp_led.h"
#include "debug_uart/bsp_debug_uart.h"
#include "led/bsp_led.h"
#include "protocol/protocol.h"

#include <stdint.h>
#include <stdbool.h>


/**
 * @brief  GPT 初始化函数
 *
 * 初始化 GPT0 定时器模块，获取计时器周期的计数次数，并将目标值发送至上位机。
 *
 * @param[in] 无
 * @retval 无
 */
void GPT_Timing_Init(void)
{
    timer_info_t info;     // 定时器信息结构体
    uint32_t temp = 0;     // 临时变量，用于存储周期时间（单位：毫秒）

    /* 初始化 GPT0 模块 */
    R_GPT_Open(&g_timer_gpt0_ctrl, &g_timer_gpt0_cfg);

    /* 获取计时器一个周期所需的计数次数 */
    R_GPT_InfoGet(&g_timer_gpt0_ctrl, &info);

    // 计算周期时间（单位：毫秒）
    temp = (uint32_t)(((float)info.period_counts / (float)CLK_GPT_COUNTS) * 1000);

    /* 向通道 1 发送目标值 */
    set_computer_value(SEND_PERIOD_CMD, CURVES_CH1, &temp, 1);

    /* 启动 GPT0 定时器 */
    R_GPT_Start(&g_timer_gpt0_ctrl);
}



/**
 * @brief  设置 GPT 定时器的周期，单位为毫秒
 * @param  ms_period: 要设置的周期，单位为毫秒
 * @retval 无
 */
void SET_BASIC_TIM_PERIOD(uint32_t ms_period)
{
    // 计算周期计数值
    uint32_t period_counts = (uint32_t)(((float)ms_period / 1000.0f) * (float)CLK_GPT_COUNTS);

    // 调用 GPT 的周期设置函数
    R_GPT_PeriodSet(&g_timer_gpt0_ctrl, period_counts);

}
