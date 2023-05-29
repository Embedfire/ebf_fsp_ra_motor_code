#ifndef _BSP_ADC_VR_H_
#define _BSP_ADC_VR_H_
#include "stdio.h"
#include "hal_data.h"

void adc_Init(void);
double Read_ADC_Voltage_Value(void);
void adc_callback(adc_callback_args_t * p_args);

#endif
