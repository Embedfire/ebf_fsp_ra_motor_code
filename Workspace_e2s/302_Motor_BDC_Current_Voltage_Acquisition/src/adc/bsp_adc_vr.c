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


void g_adc_motor_callback(adc_callback_args_t * p_args)
{
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
        //printf("\r\n%d\r\n",adc1_data);
        c0 = GET_ADC_VDC_VAL(adc1_data);
        c0 = GET_VBUS_VAL(c0);

        return c0;
}

double Read_ADC_Current_Value(void)
{
        uint16_t adc2_data;
        double c1;

        while (!scan_complete_flag)        //等待扫描完成
        {
          ;
        }

        R_ADC_B_Read(&g_adc_motor_ctrl,ADC_CHANNEL_3, &adc2_data);

        c1 = GET_ADC_VDC_VAL(adc2_data);//计算电流

        c1 = GET_ADC_CURR_VAL(c1) ;

        scan_complete_flag = false;

        return c1;
}
