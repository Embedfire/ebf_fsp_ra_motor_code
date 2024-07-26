#include "bsp_gpt_timing.h"
#include "led/bsp_led.h"
#include "adc/bsp_adc_vr.h"
#include "debug_uart/bsp_debug_uart.h"
#include "led/bsp_led.h"


// 定义移动平均滤波器的窗口大小
#define FILTER_WINDOW_SIZE 100

// 全局变量
double current_samples[FILTER_WINDOW_SIZE] = {0.0}; // 用于存储最近的100次采样值
uint16_t current_index = 0;                         // 当前采样值的索引
double current_sum = 0.0;                           // 采样值的总和
uint16_t sample_count = 0;                          // 已采样的次数


// 偏置值
double current_offset = 0.0f;

uint16_t Time = 0;



double Current_sum = 0.0,Current_avg = 0.0;



/* GPT 初始化函数 */
void GPT_Timing_Init(void)
{
    /* 初始化 GPT0 模块 */
    R_GPT_Open(&g_timer_gpt0_ctrl, &g_timer_gpt0_cfg);

    /* 启动 GPT0 定时器 */
    R_GPT_Start(&g_timer_gpt0_ctrl);

}

// 计算电流偏置值的函数
void Calculate_Current_Offset(void)
{
    double sum = 0;

    // 采集多次电流值
    for (int i = 0; i < 100; i++) {
        sum += Read_ADC_Current_Value();

    }

    // 计算平均值作为偏置值
    current_offset = (double)sum / 100;
    //printf("初始偏置值为: %.2f\n", current_offset);
}


// GPT中断回调函数
void gpt0_timing_callback(timer_callback_args_t *p_args)
{
    // 定时器溢出事件
    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        Time += 5;

        // 读取当前电流值并加入移动平均滤波器
        double current_value = Read_ADC_Current_Value();
        current_sum -= current_samples[current_index]; // 减去旧值
        current_samples[current_index] = current_value; // 更新当前值
        current_sum += current_value; // 加上新值

        // 更新索引
        current_index = (uint16_t)((current_index + 1) % FILTER_WINDOW_SIZE);

        // 计算采样次数
        if (sample_count < FILTER_WINDOW_SIZE)
        {
            sample_count++;
        }

        // 每秒计算一次平均值并打印
        if (Time == 500)
        {
            // 翻转 LED1
            LED1_TOGGLE; // 每秒翻转一次

            // 计算当前平均值
            double current_avg = current_sum / sample_count;
            printf("电压 = %.2fV, 电流 = %.2fmA\r\n", Read_ADC_Voltage_Value(), (current_avg > current_offset) ? (current_avg - current_offset) : 0);

            // 重置时间
            Time = 0;
        }
    }
}

