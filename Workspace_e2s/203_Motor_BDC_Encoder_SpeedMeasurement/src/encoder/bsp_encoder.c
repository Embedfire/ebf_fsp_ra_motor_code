#include <stdio.h>
#include <stdlib.h>
#include "bsp_encoder.h"
#include "gpt/bsp_gpt_timing.h"
#include "motor_control/bsp_motor_control.h"
#include "debug_uart/bsp_debug_uart.h"

/* 全局变量 */
volatile uint32_t pulse_period = 0;   // 脉冲数
volatile _Bool flag = true;           // 方向标志，true 为正转，false 为反转
volatile uint32_t overflow_count = 0; // 溢出计数
timer_info_t info;                    // 定时器信息，包含时钟频率等
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
    period = info.period_counts;

    /* 使能输入捕获 */
    R_GPT_Enable(&encoder_ctrl);

    /* 启动 GPT 定时器 */
    R_GPT_Start(&encoder_ctrl);
}

/* 编码器中断回调函数 */
void encoder_callback(timer_callback_args_t *p_args)
{
    switch (p_args->event)
    {
        /* 捕获事件：正转计数（捕获A事件） */
        case TIMER_EVENT_CAPTURE_A:
            flag = true; // 设置为正转方向
            //更新计数值
            pulse_period = p_args->capture + (overflow_count * period);
            break;

        /* 捕获事件：反转计数（捕获B事件） */
        case TIMER_EVENT_CAPTURE_B:
            flag = false; // 设置为反转方向
            //更新计数值
            pulse_period = p_args->capture + (overflow_count * period);
            break;

        /* 定时器溢出事件 */
        case TIMER_EVENT_CYCLE_END:
            overflow_count++; // 增加溢出计数
            break;

        default:
            break;
    }
}



/* GPT定时器中断回调函数 */
void gpt0_timing_callback(timer_callback_args_t *p_args)
{
    static uint32_t last_pulse_period = 0; // 上一次的脉冲周期
    static uint32_t new_period = 0; // 上一次的脉冲周期
    static float shaft_speed = 0.00f;
    static float output_speed = 0.00f;

    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        if (pulse_period > 0 && pulse_period != last_pulse_period)
        {

            // 更新最近速度
            new_period = (pulse_period - last_pulse_period);

            // 转轴速度计算（单位：转/秒）
            shaft_speed = SHAFT_SPEED(new_period) * 5.0f;
            // 输出轴速度计算（单位：转/秒）
            output_speed = OUTPUT_SPEED(new_period) * 5.0f;

            // 打印方向和速度
            if (flag)
            {

                    MOTOR_PRINT("方向：正转\n");
            }
            else
            {
                    MOTOR_PRINT("方向：反转\n");
            }

                MOTOR_PRINT("单位时间内有效计数值：%ld \n", new_period);
                MOTOR_PRINT("转轴处速度：%.2f 转/秒\n", shaft_speed);
                MOTOR_PRINT("输出轴速度：%.2f 转/秒\n", output_speed);
            }

           last_pulse_period = pulse_period; // 更新目前脉冲周期
       }
}




