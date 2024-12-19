#ifndef _BSP_ADC_VR_H_
#define _BSP_ADC_VR_H_
#include "stdio.h"
#include "hal_data.h"


// 系统启动时电流偏置校准次数
#define OFFSET_SAMPLES 20


#define FILTER_SIZE 150  // 定义滤波器大小

#define VREF                            3.3f     // 参考电压，主板跳帽J7部分

#define GET_ADC_VDC_VAL(val)            ((float)val/(float)65536.0*VREF)
#define GET_VBUS_VAL(val)               (((float)val - (float)1.24) * (float)37.0)// 获取测试电压（偏置电压1.24，放大倍数为37）


#define GET_ADC_CURR_VAL(val)           (((float)val)/(float)8.0/(float)0.02*(float)1000.0)          // 得到电流值，电压放大8倍，0.02是采样电阻，单位mA。

void ADC_DMAC_Init(void);


#endif
