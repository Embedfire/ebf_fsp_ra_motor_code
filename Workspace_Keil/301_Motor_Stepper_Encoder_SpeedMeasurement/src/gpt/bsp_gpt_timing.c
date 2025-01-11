#include "bsp_gpt_timing.h"
#include "led/bsp_led.h"
#include "debug_uart/bsp_debug_uart.h"
#include "led/bsp_led.h"


#include <stdint.h>
#include <stdbool.h>


/* GPT 初始化函数 */
void GPT_Timing_Init(void)
{
    /* 初始化 GPT0 模块 */
    R_GPT_Open(&g_timer_gpt0_ctrl, &g_timer_gpt0_cfg);

    /* 启动 GPT0 定时器 */
    R_GPT_Start(&g_timer_gpt0_ctrl);

}



