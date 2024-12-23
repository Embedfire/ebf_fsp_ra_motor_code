#ifndef _BSP_ADC_VR_H_
#define _BSP_ADC_VR_H_
#include "stdio.h"
#include "hal_data.h"


#define VREF                            3.3f     // 参考电压，主板跳帽J7部分

#define GET_ADC_VDC_VAL(val)            ((float)val/(float)4096.0f*VREF)
#define GET_VBUS_VAL(val)               ((float)val * (float)37.0)      // 获取测试电压（偏置电压1.24(已处理)，放大倍数为37）

void adc_Init(void);
double get_ntc_r_val(void);
double get_ntc_t_val(void);
double Read_ADC_Voltage_Value(void);
double Read_ADC_Temp_Value(void);


#endif
