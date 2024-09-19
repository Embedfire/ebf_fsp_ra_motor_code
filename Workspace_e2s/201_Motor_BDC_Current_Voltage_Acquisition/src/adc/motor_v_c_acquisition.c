#include <adc/motor_v_c_acquisition.h>
#include "debug_uart/bsp_debug_uart.h"




volatile bool scan_complete_flag = false;//adc读取完成标志位

/**
 * @brief 初始化ADC模块
 *
 * 该函数用于初始化ADC模块，包括打开ADC、配置扫描组和启动扫描组。
 *
 * @param[in] 无
 * @return 无
 * @note 在调用此函数之前，请确保ADC配置结构体已正确设置。
 */
void adc_Init(void)
{
    fsp_err_t err = FSP_SUCCESS; // 定义错误码，初始化为成功状态

    // 初始化ADC模块
    err = R_ADC_B_Open(&g_adc_motor_ctrl, &g_adc_motor_cfg);
    // 检查是否成功打开ADC，失败则断言
    assert(FSP_SUCCESS == err);

    // 配置ADC扫描组
    err = R_ADC_B_ScanCfg(&g_adc_motor_ctrl, &g_adc_motor_scan_cfg);
    // 检查扫描配置是否成功，失败则断言
    assert(FSP_SUCCESS == err);

    // 启动ADC扫描组
    err = R_ADC_B_ScanGroupStart(&g_adc_motor_ctrl, ADC_GROUP_MASK_ALL);
    // 检查扫描组启动是否成功，失败则断言
    assert(FSP_SUCCESS == err);
}

/**
 * @brief ADC回调函数
 *
 * 该函数在ADC扫描完成时被调用，用于设置扫描完成标志。
 *
 * @param[in] p_args 指向ADC回调参数的指针
 * @return 无
 * @note 此函数由ADC驱动程序自动调用。
 */
void g_adc_motor_callback(adc_callback_args_t * p_args)
{
    // 使用参数宏，标记未使用的参数
    FSP_PARAMETER_NOT_USED(p_args);
    // 扫描完成标志置为真
    scan_complete_flag = true;
}

/**
 * @brief 读取ADC电压值
 *
 * 该函数从ADC通道1读取电压值，并将其转换为实际电压值。
 *
 * @param[in] 无
 * @return 返回读取到的电压值
 * @note 此函数在调用前需确保ADC扫描已完成。
 */
double Read_ADC_Voltage_Value(void)
{
    uint16_t adc1_data; // 存储ADC读取到的数据
    double c0; // 存储转换后的电压值

    // 等待扫描完成
    while (!scan_complete_flag)
    {
        ; // 空循环，直到扫描完成标志为真
    }

    // 从ADC通道1读取数据
    R_ADC_B_Read(&g_adc_motor_ctrl, ADC_CHANNEL_1, &adc1_data);

    // 将ADC值转换为电压值
    c0 = GET_ADC_VDC_VAL(adc1_data); // 获取直流电压值
    c0 = GET_VBUS_VAL(c0); // 进一步获取总线电压值

    // 返回电压值
    return c0;
}

/**
 * @brief 读取ADC电流值
 *
 * 该函数从ADC通道3读取电流值，并将其转换为实际电流值。
 *
 * @param[in] 无
 * @return 返回读取到的电流值
 * @note 此函数在调用前需确保ADC扫描已完成。
 */
double Read_ADC_Current_Value(void)
{
    uint16_t adc2_data; // 存储ADC读取到的数据
    double c1; // 存储转换后的电流值

    // 等待扫描完成
    while (!scan_complete_flag)
    {
        ; // 空循环，直到扫描完成标志为真
    }

    // 从ADC通道3读取数据
    R_ADC_B_Read(&g_adc_motor_ctrl, ADC_CHANNEL_3, &adc2_data);

    // 计算电流
    c1 = GET_ADC_VDC_VAL(adc2_data); // 获取直流电压值作为电流数据
    c1 = GET_ADC_CURR_VAL(c1); // 进一步获取电流值

    // 置扫描完成标志为假，以便进行下一次扫描
    scan_complete_flag = false;

    // 返回电流值
    return c1;
}
