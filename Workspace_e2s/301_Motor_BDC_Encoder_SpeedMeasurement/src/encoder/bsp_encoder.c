#include "bsp_encoder.h"
#include "led/bsp_led.h"
#include "motor_control/bsp_motor_control.h"
#include "debug_uart/bsp_debug_uart.h"

_Bool flag = true;

timer_info_t info;  //用于获取定时器参数信息
uint32_t period;    //输入捕获计数器的计数周期

/* 初始化编码器 */
void initEncoder(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /* 打开编码器 */
    err = R_GPT_Open(&encoder_ctrl, &encoder_cfg);
    assert(FSP_SUCCESS == err);

    /*获取当前参数*/
    (void) R_GPT_InfoGet(&encoder_ctrl, &info);
    /* 获取计数周期：GPT的一个周期的计数次数 */
    period = (uint32_t)info.period_counts;

    /* 使能输入捕获 */
    R_GPT_Enable(&encoder_ctrl);

}


uint32_t pulse_period;          // 两次捕获的差值，一个脉冲的周期数
uint32_t phase_a_frequency;     // A相频率值
volatile bool capture_ready = false;  // 捕获完成标志，表示已完成一次有效捕获


/* 编码器中断回调函数 */
void encoder_callback(timer_callback_args_t *p_args)
{
    // 静态局部变量用于存储捕获值和状态，确保在函数调用之间保持值
    static uint32_t capture_first = 0;  // 第一次捕获值
    static uint32_t capture_second = 0; // 第二次捕获值
    static uint8_t capture_flag = 0;    // 捕获标志，0表示未捕获，1表示已捕获
    static uint32_t overflow_count = 0; // 溢出计数，记录捕获之间的定时器溢出次数

    switch (p_args->event)
    {
        /* 捕获事件 正转计数 */
        case TIMER_EVENT_CAPTURE_A:
            flag = true;
            if (capture_flag == 0)
            {
                // 第一次捕获事件，记录捕获值并重置溢出计数
                capture_first = p_args->capture;
                overflow_count = 0;
                capture_flag = 1;
            }
            else if (capture_flag == 1)
            {
                // 第二次捕获事件，计算捕获周期并计算频率
                capture_second = p_args->capture + overflow_count * TIMER_MAX_COUNT;
                pulse_period = capture_second - capture_first;
                phase_a_frequency = info.clock_frequency / (pulse_period*1);

                // 重置捕获标志和溢出计数，并设置捕获完成标志
                overflow_count = 0;
                capture_flag = 0;
                capture_ready = true;
            }
            break;

        /* 捕获事件 反转计数 */
        case TIMER_EVENT_CAPTURE_B:
            flag = false;
            if (capture_flag == 0)
            {
                // 第一次捕获事件，记录捕获值并重置溢出计数
                capture_first = p_args->capture;
                overflow_count = 0;
                capture_flag = 1;
            }
            else if (capture_flag == 1)
            {
                // 第二次捕获事件，计算捕获周期并计算频率
                capture_second = p_args->capture + overflow_count * TIMER_MAX_COUNT;
                pulse_period = capture_second - capture_first;
                phase_a_frequency = info.clock_frequency / (pulse_period*1);

                // 重置捕获标志和溢出计数，并设置捕获完成标志
                overflow_count = 0;
                capture_flag = 0;
                capture_ready = true;
            }
            break;
            break;

        /* 计数溢出事件 */
        case TIMER_EVENT_CYCLE_END:
            // 捕获间定时器溢出事件，增加溢出计数
            overflow_count++;
            break;

        default:
            break;
    }
}


