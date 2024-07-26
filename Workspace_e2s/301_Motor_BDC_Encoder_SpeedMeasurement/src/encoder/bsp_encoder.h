#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H


#include "hal_data.h"



#define TIMER_MAX_COUNT 65536  // 定时器最大计数值


 /* 编码器物理分辨率 */
 #define ENCODER_RESOLUTION                  16.00

 /* 减速电机减速比 */
 #define REDUCTION_RATIO                     30.00

void initEncoder(void);

#endif