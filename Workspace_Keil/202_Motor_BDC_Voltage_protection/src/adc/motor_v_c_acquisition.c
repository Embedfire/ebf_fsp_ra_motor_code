#include <adc/motor_v_c_acquisition.h>
#include <motor_control/bsp_motor_control.h>
#include "debug_uart/bsp_debug_uart.h"
#include "beep/bsp_beep.h"

fsp_err_t err = FSP_SUCCESS; // 定义错误码，初始化为成功状态

uint32_t adc_result_buffer[4];//初始化DMAC传输目的地
uint32_t current_offset = 0;  // 电流偏置全局变量

/**
 * @brief 初始化ADC与DMAC模块
 *
 * 该函数用于初始化ADC模块，包括打开ADC、配置扫描组、校准ADC和启动扫描组，以及配置、初始化DMAC模块。
 *
 * @param[in] 无
 * @return 无
 * @note 在调用此函数之前，请确保ADC配置结构体已正确设置。
 */
void ADC_DMAC_Init(void)
{

    // 初始化ADC模块
    err = R_ADC_B_Open(&g_adc_motor_ctrl, &g_adc_motor_cfg);
    //检查是否成功
    assert(FSP_SUCCESS == err);

    // 配置ADC扫描组
    err = R_ADC_B_ScanCfg(&g_adc_motor_ctrl, &g_adc_motor_scan_cfg);
    //检查是否成功
    assert(FSP_SUCCESS == err);

    /* 进行ADC校准 */
    err = R_ADC_B_Calibrate(&g_adc_motor_ctrl, NULL);
    assert(FSP_SUCCESS == err); // 确保校准初始化成功。

    /* 等待校准完成 */
    adc_status_t status = {.state = ADC_STATE_CALIBRATION_IN_PROGRESS}; // 初始化状态为“校准中”
    while ((ADC_STATE_IDLE != status.state) && // 检查ADC状态是否已空闲
           (FSP_SUCCESS == err)) // 确保没有错误发生
    {
        R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS); // 延迟1毫秒以避免忙等待
        err = R_ADC_B_StatusGet(&g_adc_motor_ctrl, &status); // 获取当前ADC状态
    }
    // 检查是否成功
    assert(FSP_SUCCESS == err);

    /*以下配置DMAC*/

    // 数据源为R_ADC_B数据寄存器
    g_adc_transfer_cfg.p_info->p_src = (void*)&R_ADC_B->ADDR[0];
    // 数据目标为结果缓冲区
    g_adc_transfer_cfg.p_info->p_dest = &adc_result_buffer[0];

    //配置完成后打开DMAC实例
    err = R_DMAC_Open(&g_adc_transfer_ctrl, &g_adc_transfer_cfg);
    // 检查是否成功
    assert(FSP_SUCCESS == err);

    //使能DMAC
    err = R_DMAC_Enable(&g_adc_transfer_ctrl);
    // 检查是否成功
    assert(FSP_SUCCESS == err);

    //启动ADC扫描组
    err = R_ADC_B_ScanGroupStart(&g_adc_motor_ctrl, ADC_GROUP_MASK_0);
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
        switch (p_args->event)
        {
            /*组扫描完成*/
            case ADC_EVENT_SCAN_COMPLETE:
                /*重置传输源、传输目的地*/
                err = R_DMAC_Reset(
                          &g_adc_transfer_ctrl,       // DMA 控制结构体指针
                          (void*)&R_ADC_B->ADDR[0],   // 源地址（ADC 数据寄存器）
                          &adc_result_buffer[0],      // 目标地址（结果缓冲区）
                          1                           // 传输块大小
                      );

                if (FSP_SUCCESS != err)
                {
                    __BKPT(1); // 触发断点，用于调试错误
                }
                break;
             case  ADC_EVENT_LIMIT_CLIP:

                 Motor_Control_Stop();   // 停止电机

                 R_GPT_Stop(&g_timer_gpt0_ctrl);  //停止定时器

                 //蜂鸣器提醒
                 BUZZER_ON;
                 R_BSP_SoftwareDelay (200, BSP_DELAY_UNITS_MILLISECONDS);
                 BUZZER_OFF;

                 //检查是欠压还是过压
                 if(adc_result_buffer[1] <= 0x7529){
                 MOTOR_PRINT("当前欠压，电机电压已不足10V，已停止电机，请复位开发板再试\r\n");
                 }else{
                 MOTOR_PRINT("当前过压，电机电压已超过20V，已停止电机，请复位开发板再试\r\n");
                 }
                 __BKPT(1); // 触发断点，用于调试错误
                 break;
            default:
               break;
        }
}


/********对电流进行平滑滤波得到准确电流**********/

uint32_t current_filter_buffer[FILTER_SIZE];   // 电流数据的滤波缓冲区
uint32_t filter_index = 0;                     // 当前滤波索引
uint32_t filtered_current = 0;                 // 滤波后的电流值
uint32_t current_sum = 0;                      // 用于计算平均值的总和
int i = 0;                                     // 用于计算偏置值

//DMAC中断，每传输完一次数据就进行中断处理
void dmac_callback (dmac_callback_args_t * cb_data)
{
    FSP_PARAMETER_NOT_USED(cb_data);

    // 等待20次数据，记录最大偏置值
    if (i <= OFFSET_SAMPLES) {
        // 第一次初始化
        if (i == 0) {
            current_offset = adc_result_buffer[3];
        }
        else {
            // 比较并保存最大值
            if (adc_result_buffer[3] > current_offset) {
                current_offset = adc_result_buffer[3];
            }
        }
        i++;
    }

    //确保偏置值记录完成
    else
    {
        // 将新的电流值存入滤波缓冲区
        current_sum -= current_filter_buffer[filter_index];             // 减去旧的值
        current_filter_buffer[filter_index] = adc_result_buffer[3];     // 存入新的电流值
        current_sum += adc_result_buffer[3];                            // 加上新的值

        // 更新滤波器索引，达到滤波器大小时回绕
        filter_index = (filter_index + 1) % FILTER_SIZE;

        // 计算并保存滤波后的电流值（简单的移动平均）
        filtered_current = current_sum / FILTER_SIZE;

        //减去偏置值并防止负值
        if(filtered_current >= current_offset){
            filtered_current -= current_offset;
        }else
            filtered_current = 0;
    }
}


// GPT中断回调函数
void gpt0_timing_callback(timer_callback_args_t *p_args)
{
    // 定时器溢出事件
    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        // 先计算电压和电流
        float vbus = GET_VBUS_VAL(GET_ADC_VDC_VAL((float)adc_result_buffer[1]));       // 电压计算
        float current = GET_ADC_CURR_VAL(GET_ADC_VDC_VAL((float)(filtered_current)));  // 电流计算

        // 然后打印
        MOTOR_PRINT("电压 = %.2fV, 电流 = %dmA\r\n", vbus, (int)current);
    }
}


