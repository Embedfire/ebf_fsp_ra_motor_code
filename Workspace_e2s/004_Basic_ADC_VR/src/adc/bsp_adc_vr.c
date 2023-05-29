#include "bsp_adc_vr.h"


void adc_Init(void)
{
      fsp_err_t err = FSP_SUCCESS;
       //ADC初始化
      err = R_ADC_B_Open(&g_adc0_ctrl, &g_adc0_cfg);
      assert(FSP_SUCCESS == err);

      err = R_ADC_B_ScanCfg(&g_adc0_ctrl, &g_adc0_scan_cfg);
      assert(FSP_SUCCESS == err);

      err = R_ADC_B_ScanGroupStart(&g_adc0_ctrl, ADC_GROUP_MASK_ALL);
      assert(FSP_SUCCESS == err);
}


volatile bool scan_complete_flag = false;//adc读取完成标志位


void adc015_callback(adc_callback_args_t * p_args)
{
   FSP_PARAMETER_NOT_USED(p_args);
   scan_complete_flag = true;
}

double Read_ADC_Voltage_Value(void)
{

    uint16_t adc_data;
       double c0;


       //(void)R_ADC_B_ScanStart(&g_adc015_ctrl);
       R_ADC_B_ScanGroupStart(&g_adc0_ctrl, ADC_GROUP_MASK_ALL);
       scan_complete_flag = false;
       while (!scan_complete_flag)                  //等待扫描完成
       {
          ;
       }

       R_ADC_B_Read(&g_adc0_ctrl,ADC_CHANNEL_15, &adc_data);
       c0 = (double)(adc_data*3.3/4095);
       return c0;
}
