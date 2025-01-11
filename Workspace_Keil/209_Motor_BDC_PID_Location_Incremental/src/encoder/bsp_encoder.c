#include <stdio.h>
#include <stdlib.h>
#include "bsp_encoder.h"
#include "gpt/bsp_gpt_timing.h"
#include "motor_control/bsp_motor_control.h"
#include "debug_uart/bsp_debug_uart.h"


/* 全局变量 */
volatile int32_t pulse_period = 0;   // 脉冲数
volatile _Bool flag = true;           // 方向标志，true 为正转，false 为反转
volatile uint32_t overflow_count = 0; // 溢出计数
timer_info_t info;                    // 定时器信息，包含时钟频率等
uint32_t period;                      //输入捕获计数器的计数周期

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
            pulse_period++;
            break;

        /* 捕获事件：反转计数（捕获B事件） */
        case TIMER_EVENT_CAPTURE_B:
            flag = false; // 设置为反转方向
            //更新计数值
            pulse_period--;
            break;

        /* 定时器溢出事件 */
        case TIMER_EVENT_CYCLE_END:
            overflow_count++; // 增加溢出计数
            break;

        default:
            break;
    }
}








