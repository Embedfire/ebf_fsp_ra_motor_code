#include "bsp_adc_vr.h"
#include "debug_uart/bsp_debug_uart.h"




volatile bool scan_complete_flag = false;//adc读取完成标志位

void adc_Init(void)
{

      fsp_err_t err = FSP_SUCCESS;
       //ADC初始化
      err = R_ADC_B_Open(&g_adc_motor_ctrl, &g_adc_motor_cfg);
      assert(FSP_SUCCESS == err);

      err = R_ADC_B_ScanCfg(&g_adc_motor_ctrl, &g_adc_motor_scan_cfg);
      assert(FSP_SUCCESS == err);

      err = R_ADC_B_ScanGroupStart(&g_adc_motor_ctrl, ADC_GROUP_MASK_ALL);
      assert(FSP_SUCCESS == err);



}

volatile bool max_flag = false;

void g_adc_motor_callback(adc_callback_args_t * p_args)
{

    if((p_args->event == ADC_EVENT_LIMIT_CLIP))
    {
        max_flag = true;
    }
    FSP_PARAMETER_NOT_USED(p_args);
    scan_complete_flag = true;


}

double Read_ADC_Voltage_Value(void)
{
        uint16_t adc1_data;
        double c0;

        while (!scan_complete_flag)         //等待扫描完成
        {
          ;
        }
        R_ADC_B_Read(&g_adc_motor_ctrl,ADC_CHANNEL_1, &adc1_data);

        c0 = GET_ADC_VDC_VAL(adc1_data);

        c0 = GET_VBUS_VAL(c0);


        return c0;
}


double Read_ADC_Temp_Value(void)
{
        uint16_t adc2_data;
        double c1;

        while (!scan_complete_flag)        //等待扫描完成
        {
          ;
        }

        R_ADC_B_Read(&g_adc_motor_ctrl,ADC_CHANNEL_3, &adc2_data);

        c1 = GET_ADC_VDC_VAL(adc2_data);//计算电流

        return c1;
}

/**
  * @brief  获取温度传感器端的电阻值
  * @param  无
  * @retval 转换得到的电阻值
  */
double get_ntc_r_val(void)
{
    double r = 0;
    double vdc = Read_ADC_Temp_Value();

    r = ((VREF - vdc) / (vdc / (float)4700.0));

    return r;
}


/**
  * @brief  获取温度传感器的温度
  * @param  无
  * @retval 转换得到的温度，单位：（℃）
  */
double get_ntc_t_val(void)
{
    double t = 0;             // 测量温度
    double Rt = 0;            // 测量电阻
    double Ka = 273.15;       // 0℃ 时对应的温度（开尔文）
    double R25 = 10000.0;     // 25℃ 电阻值
    double T25 = Ka + 25;     // 25℃ 时对应的温度（开尔文）
    double B = 3950.0;        /* B-常数：B = ln(R25 / Rt) / (1 / T – 1 / T25)，
                             其中 T = 25 + 273.15 */

    Rt = get_ntc_r_val();    // 获取当前电阻值

    t = B * T25 / (B + log(Rt / R25) * T25) - Ka ;    // 使用公式计算

    return t;
}

