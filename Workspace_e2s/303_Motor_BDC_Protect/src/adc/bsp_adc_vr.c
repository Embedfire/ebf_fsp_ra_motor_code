#include "bsp_adc_vr.h"
#include "debug_uart/bsp_debug_uart.h"
#include <motor_control/bsp_motor_control.h>
#include "beep/bsp_beep.h"
#include "led/bsp_led.h"

volatile bool scan_complete_flag = false;//adc读取完成标志位

/**
 * @brief ADC初始化函数
 *
 * 该函数初始化ADC模块并开始扫描所有通道。
 */
void adc_Init(void)
{
    fsp_err_t err = FSP_SUCCESS; // 存储函数调用的返回值

    // ADC初始化
    err = R_ADC_B_Open(&g_adc_motor_ctrl, &g_adc_motor_cfg);
    assert(FSP_SUCCESS == err);

    // 配置ADC扫描
    err = R_ADC_B_ScanCfg(&g_adc_motor_ctrl, &g_adc_motor_scan_cfg);
    assert(FSP_SUCCESS == err);

    // 开始扫描所有通道
    err = R_ADC_B_ScanGroupStart(&g_adc_motor_ctrl, ADC_GROUP_MASK_ALL);
    assert(FSP_SUCCESS == err);
}


/**
 * @brief ADC电机控制回调函数
 *
 * 这个函数会在ADC事件发生时被调用。如果检测到ADC限幅裁剪事件，
 * 会停止电机和定时器，并通过蜂鸣器和LED提醒用户复位开发板。
 *
 * @param p_args 指向ADC回调参数的指针
 */
void g_adc_motor_callback(adc_callback_args_t * p_args)
{
    // 如果发生了ADC事件限幅裁剪(过压、欠压、过流)
    if(p_args->event == ADC_EVENT_LIMIT_CLIP)
    {
        Motor_Control_Stop(); // 停止电机
        R_GPT_Stop(&g_timer_gpt0_ctrl); // 停止定时器

        /* 蜂鸣器响一声提醒用户 */
        BUZZER_ON;
        R_BSP_SoftwareDelay(100, BSP_DELAY_UNITS_MILLISECONDS);
        BUZZER_OFF;
        /*判断是过压、欠压、过流哪种情况，还未写*/
        printf("********当前状态：过压********\r\n");
        printf("********请复位开发板再试********\r\n");
        while(1)
        {
            LED1_TOGGLE; // 红灯闪烁
            R_BSP_SoftwareDelay(50, BSP_DELAY_UNITS_MILLISECONDS);
        }
    }

    scan_complete_flag = true; // 设置扫描完成标志
}

/**
 * @brief 读取ADC电压值
 *
 * 该函数读取ADC通道1的数据，计算并返回对应的电压值。
 *
 * @return 返回计算后的电压值
 */
double Read_ADC_Voltage_Value(void)
{
    uint16_t adc1_data; // 存储ADC数据
    double c0; // 存储计算的电压值

    // 等待扫描完成
    while (!scan_complete_flag)
    {
        ;
    }

    // 读取ADC通道1的数据
    R_ADC_B_Read(&g_adc_motor_ctrl, ADC_CHANNEL_1, &adc1_data);
    printf("\r\n%d\r\n", adc1_data);

    // 计算电压值
    c0 = GET_ADC_VDC_VAL(adc1_data);
    c0 = GET_VBUS_VAL(c0);

    return c0; // 返回计算后的电压值
}


/**
 * @brief 读取ADC电流值
 *
 * 该函数读取ADC通道3的数据，计算并返回对应的电流值。
 *
 * @return 返回计算后的电流值
 */
double Read_ADC_Current_Value(void)
{
    uint16_t adc2_data; // 存储ADC数据
    double c1; // 存储计算的电流值

    // 等待扫描完成
    while (!scan_complete_flag)
    {
        ;
    }

    // 读取ADC通道3的数据
    R_ADC_B_Read(&g_adc_motor_ctrl, ADC_CHANNEL_3, &adc2_data);

    // 计算电流值
    c1 = GET_ADC_VDC_VAL(adc2_data);
    c1 = GET_ADC_CURR_VAL(c1);

    scan_complete_flag = false;

    return c1; // 返回计算后的电流值
}

