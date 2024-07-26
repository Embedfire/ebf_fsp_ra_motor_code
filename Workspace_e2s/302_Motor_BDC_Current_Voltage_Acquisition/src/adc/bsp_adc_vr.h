#ifndef _BSP_ADC_VR_H_
#define _BSP_ADC_VR_H_
#include "stdio.h"
#include "hal_data.h"


#define VREF                            3.3f     // 参考电压，主板跳帽J7部分

#define GET_ADC_VDC_VAL(val)            ((float)val/(float)65535.0f*VREF)
#define GET_VBUS_VAL(val)               ((float)val * (float)37.0)      // 获取测试电压（偏置电压1.24(已处理)，放大倍数为37）
#define GET_ADC_CURR_VAL(val)           (((float)val)/(float)8.0/(float)0.02*(float)1000.0)          // 得到电流值，电压放大8倍，0.02是采样电阻，单位mA。

void adc_Init(void);
double Read_ADC_Voltage_Value(void);
double Read_ADC_Current_Value(void);
int32_t get_curr_val(void);


#endif
