#ifndef GPT_BSP_GPT_TIMING_H_
#define GPT_BSP_GPT_TIMING_H_

#include <hal_data.h>




void GPT_Timing_Init(void);
void Deal_Current_U_Data(void);
void Deal_Current_V_Data(void);
void Deal_Current_W_Data(void);
void Calculate_Current_Offset(void) ;

#endif /* GPT_BSP_GPT_TIMING_H_ */
