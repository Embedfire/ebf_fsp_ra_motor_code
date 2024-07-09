#ifndef _BSP_ADC_VR_H_
#define _BSP_ADC_VR_H_
#include "stdio.h"
#include "hal_data.h"



#define GET_VBUS_VAL(val)               (((float)val-(float)1.24) * (float)37.0)      // 获取测试电压（偏置电压1.24，放大倍数为37）

void adc_Init(void);
double Read_ADC_Voltage_Value(void);


#endif
