#include "bsp_gpt_timing.h"
#include "led/bsp_led.h"
#include "adc/bsp_adc_vr.h"
#include "debug_uart/bsp_debug_uart.h"
#include "led/bsp_led.h"

// 定义移动平均滤波器的窗口大小
#define FILTER_WINDOW_SIZE 100


/* GPT 初始化函数 */
void GPT_Timing_Init(void)
{
    /* 初始化 GPT0 模块 */
    R_GPT_Open(&g_timer_gpt0_ctrl, &g_timer_gpt0_cfg);

    /* 启动 GPT0 定时器 */
    R_GPT_Start(&g_timer_gpt0_ctrl);

}

// 偏置值
double u_current_offset = 0.0f;
double v_current_offset = 0.0f;
double w_current_offset = 0.0f;
// 计算电流偏置值的函数
void Calculate_Current_Offset(void)
{
    double u_sum = 0;
    double v_sum = 0;
    double w_sum = 0;

    // 采集多次电流值
    for (int i = 0; i < 100; i++)
    {
        u_sum += Read_ADC_Current_U_Value();
        v_sum += Read_ADC_Current_V_Value();
        w_sum += Read_ADC_Current_W_Value();
    }

    // 计算平均值作为偏置值
    u_current_offset = (double)u_sum / 100;
    v_current_offset = (double)v_sum / 100;
    w_current_offset = (double)w_sum / 100;

    printf("初始u偏置值为: %.2f\n", u_current_offset);
    printf("初始v偏置值为: %.2f\n", v_current_offset);
    printf("初始w偏置值为: %.2f\n", w_current_offset);
}


// 全局变量
double u_current_sum = 0.0;                         // 采样值的总和
uint16_t u_sample_count = 0;                        // 已采样的次数
void Deal_Current_U_Data(void)
{
    static double u_current_samples[FILTER_WINDOW_SIZE] = {0.0}; // 用于存储最近的100次采样值
    static uint16_t u_current_index = 0;                       // 当前采样值的索引

    // 读取当前电流值并加入移动平均滤波器
    double u_current_value = Read_ADC_Current_U_Value();
    u_current_sum -= u_current_samples[u_current_index]; // 减去旧值
    u_current_samples[u_current_index] = u_current_value; // 更新当前值
    u_current_sum += u_current_value; // 加上新值

    // 更新索引
    u_current_index = (uint16_t)((u_current_index + 1) % FILTER_WINDOW_SIZE);

    // 计算采样次数
    if (u_sample_count < FILTER_WINDOW_SIZE)
    {
        u_sample_count++;
    }
}

// 全局变量
double v_current_sum = 0.0;                         // 采样值的总和
uint16_t v_sample_count = 0;                        // 已采样的次数
void Deal_Current_V_Data(void)
{
    static double v_current_samples[FILTER_WINDOW_SIZE] = {0.0}; // 用于存储最近的100次采样值
    static uint16_t v_current_index = 0;                       // 当前采样值的索引

    // 读取当前电流值并加入移动平均滤波器
    double v_current_value = Read_ADC_Current_V_Value();
    v_current_sum -= v_current_samples[v_current_index]; // 减去旧值
    v_current_samples[v_current_index] = v_current_value; // 更新当前值
    v_current_sum += v_current_value; // 加上新值

    // 更新索引
    v_current_index = (uint16_t)((v_current_index + 1) % FILTER_WINDOW_SIZE);

    // 计算采样次数
    if (v_sample_count < FILTER_WINDOW_SIZE)
    {
        v_sample_count++;
    }
}

// 全局变量
double w_current_sum = 0.0;                         // 采样值的总和
uint16_t w_sample_count = 0;                        // 已采样的次数
void Deal_Current_W_Data(void)
{
    static double w_current_samples[FILTER_WINDOW_SIZE] = {0.0}; // 用于存储最近的100次采样值
    static uint16_t w_current_index = 0;                       // 当前采样值的索引

    // 读取当前电流值并加入移动平均滤波器
    double w_current_value = Read_ADC_Current_W_Value();
    w_current_sum -= w_current_samples[w_current_index]; // 减去旧值
    w_current_samples[w_current_index] = w_current_value; // 更新当前值
    w_current_sum += w_current_value; // 加上新值

    // 更新索引
    w_current_index = (uint16_t)((w_current_index + 1) % FILTER_WINDOW_SIZE);

    // 计算采样次数
    if (w_sample_count < FILTER_WINDOW_SIZE)
    {
        w_sample_count++;
    }
}



uint16_t Time = 0;
// GPT中断回调函数
void gpt0_timing_callback(timer_callback_args_t *p_args)
{
    // 定时器溢出事件
    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        Time += 5;

        Deal_Current_U_Data();
        Deal_Current_V_Data();
        Deal_Current_W_Data();

        // 每秒计算一次平均值并打印
        if (Time == 100)
        {
            double u_current_avg = (u_current_sum / u_sample_count - u_current_offset);
            double v_current_avg = (v_current_sum / v_sample_count - v_current_offset);
            double w_current_avg = (w_current_sum / w_sample_count - w_current_offset);

            printf("V = %.2fV  Temp = %.1f  u = %.0fmA  v = %.0fmA  w = %.0fmA\r\n",Read_ADC_Voltage_Value(),get_ntc_t_val(),u_current_avg,v_current_avg,w_current_avg);

            // 重置时间
            Time = 0;
        }
    }
}

