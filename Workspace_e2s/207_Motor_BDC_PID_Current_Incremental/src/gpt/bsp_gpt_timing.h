#ifndef GPT_BSP_GPT_TIMING_H_
#define GPT_BSP_GPT_TIMING_H_

#include <hal_data.h>

#define CLK_GPT_COUNTS      120000000

void GPT_Timing_Init(void);
void SET_BASIC_TIM_PERIOD(uint32_t ms_period);


#endif /* GPT_BSP_GPT_TIMING_H_ */
