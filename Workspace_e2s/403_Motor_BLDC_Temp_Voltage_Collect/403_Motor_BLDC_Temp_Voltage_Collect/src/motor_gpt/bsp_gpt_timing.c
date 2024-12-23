#include "bsp_gpt_timing.h"
#include "led/bsp_led.h"
#include "adc/bsp_adc_vr.h"
#include "debug_uart/bsp_debug_uart.h"
#include "led/bsp_led.h"



/* GPT 初始化函数 */
void GPT_Timing_Init(void)
{
    /* 初始化 GPT0 模块 */
    R_GPT_Open(&g_timer_gpt0_ctrl, &g_timer_gpt0_cfg);

    /* 启动 GPT0 定时器 */
    R_GPT_Start(&g_timer_gpt0_ctrl);

}


uint16_t Time = 0;

// GPT中断回调函数
void gpt0_timing_callback(timer_callback_args_t *p_args)
{
    // 定时器溢出事件
    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        Time += 5;


        // 每秒计算一次平均值并打印
        if (Time == 100)
        {
            printf("电压 = %.2fV, 温度=%0.1f°\r\n", Read_ADC_Voltage_Value(),get_ntc_t_val());

            // 重置时间
            Time = 0;
        }
    }
}

