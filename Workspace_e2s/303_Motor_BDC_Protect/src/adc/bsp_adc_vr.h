#ifndef _BSP_ADC_VR_H_
#define _BSP_ADC_VR_H_
#include "stdio.h"
#include "hal_data.h"


// 参考电压，主板跳帽J7部分
#define VREF                            3.3f

// 计算ADC采集到的电压值
#define GET_ADC_VDC_VAL(val)            ((float)val/(float)4096.0f*VREF)

// 获取测试电压（偏置电压1.24已处理，放大倍数为37）
#define GET_VBUS_VAL(val)               ((float)val * (float)37.0)

// 计算电流值，电压放大8倍，0.02是采样电阻，单位mA
#define GET_ADC_CURR_VAL(val)           (((float)val)/(float)8.0/(float)0.02*(float)1000.0)




// ADC初始化函数
void adc_Init(void);
// 读取ADC电压值函数
double Read_ADC_Voltage_Value(void);
// 读取ADC电流值函数
double Read_ADC_Current_Value(void);
// 获取当前电流值函数
int32_t get_curr_val(void);



#endif
