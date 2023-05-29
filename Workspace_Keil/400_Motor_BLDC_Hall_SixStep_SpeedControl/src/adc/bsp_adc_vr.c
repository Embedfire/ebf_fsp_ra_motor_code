#include "bsp_adc_vr.h"


void adc_Init(void)
{
      fsp_err_t err = FSP_SUCCESS;
       //ADC初始化
      err = R_ADC_B_Open(&adc0_ctrl, &adc0_cfg);
      assert(FSP_SUCCESS == err);

      err = R_ADC_B_ScanCfg(&adc0_ctrl, &adc0_scan_cfg);
      assert(FSP_SUCCESS == err);

      err = R_ADC_B_ScanGroupStart(&adc0_ctrl, ADC_GROUP_MASK_ALL);
      assert(FSP_SUCCESS == err);
}


volatile bool scan_complete_flag = false;//adc读取完成标志位


void adc0_callback(adc_callback_args_t * p_args)
{
   FSP_PARAMETER_NOT_USED(p_args);
   scan_complete_flag = true;
}

uint16_t Read_ADC_Voltage_Value(void)
{

	   uint16_t adc_data;
       uint16_t c0;

       R_ADC_B_ScanGroupStart(&adc0_ctrl, ADC_GROUP_MASK_ALL);
       scan_complete_flag = false;
       while (!scan_complete_flag)                  //等待扫描完成
       {
          ;
       }

       R_ADC_B_Read(&adc0_ctrl,ADC_CHANNEL_15, &adc_data);
       c0 = adc_data;
       return c0;
}
