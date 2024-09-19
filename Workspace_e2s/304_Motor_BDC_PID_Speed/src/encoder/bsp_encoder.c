#include "bsp_encoder.h"
#include "led/bsp_led.h"
#include "gpt/bsp_gpt_pwm_output.h"
#include "motor_control/bsp_motor_control.h"
#include "debug_uart/bsp_debug_uart.h"
#include "protocol/protocol.h"
#include "pid/bsp_pid.h"


extern _Bool motor_dir;          // 电机方向标志，`true`表示正转，`false`表示反转
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

/**
  * @brief  PID算法实现，用于根据实际转轴速度和预期速度计算PWM控制值
  * @param  real_speed  实时转轴速度
  * @param  target_speed 预期转轴速度
  * @retval 通过PID计算后的PWM控制值
  */
float PID_Control(float real_speed, float target_speed)
{
    // 定义PID结构体变量
    static struct {
        float Kp;          // 比例系数
        float Ki;          // 积分系数
        float Kd;          // 微分系数
        float prev_error;  // 上一次的误差
        float integral;    // 误差积分值
    } pid = {0.5, 0.2, 0.05, 0, 0}; // 初始化PID参数

    float error;       // 当前误差
    float derivative;  // 误差的微分
    float output;      // PID输出值

    // 计算当前误差
    error = target_speed - real_speed;

    // 误差累积
    pid.integral += error;

    // 计算误差的微分
    derivative = error - pid.prev_error;

    // PID算法计算输出
    output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;

    // 更新上一次误差
    pid.prev_error = error;

    // 返回PID计算的输出值，通常用于控制PWM占空比
    return output;
}


uint32_t pulse_period;          // 两次捕获的差值，一个脉冲的周期数
uint32_t phase_a_frequency;     // A相频率值

uint16_t value_speed;

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

                //set_computer_value(SEND_FACT_CMD, CURVES_CH1, SHAFT_SPEED(phase_a_frequency), 1);

                //经过PID计算，将占空比进行控制（）
                Motor_Control_SetDirAndDuty(motor_dir, PID_Control(SHAFT_SPEED(phase_a_frequency),value_speed)); // 设置电机方向和占空比


                // 重置捕获标志和溢出计数，并设置捕获完成标志
                overflow_count = 0;
                capture_flag = 0;
                capture_ready = true;
            }
            break;

        /* 捕获事件 反转计数 */
        case TIMER_EVENT_CAPTURE_B:
            // 根据需要处理反转计数事件，目前为空
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




/*还是需要定时，暂时注释掉*/
// /* 定时器0中断回调函数 */
//void time0_callback(timer_callback_args_t * p_args)
//{
//	R_GPT_Stop(&encoder_ctrl);
//    /* 定时周期结束事件 */
//    if (TIMER_EVENT_CYCLE_END == p_args->event)
//    {
//		x = encoder_ctrl.p_reg->GTCNT;
//		dc_motor_s.now_speed = (float)((x * 20 * 60 / 4.0 / 960.0) * flag);
//		R_GPT_CounterSet(&encoder_ctrl, 0);
//    }
//	R_GPT_Start(&encoder_ctrl);
//}

