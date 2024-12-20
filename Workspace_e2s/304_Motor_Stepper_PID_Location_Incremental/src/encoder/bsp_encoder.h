#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H


#include "hal_data.h"


/* 定时器最大计数值 */
#define TIMER_MAX_COUNT 65536U

/* 编码器接口倍频数 */
#define ENCODER_MODE                         TIM_ENCODERMODE_TI12

/* 编码器物理分辨率 */
#define ENCODER_RESOLUTION                     1000.0f

/* 经过倍频之后的总分辨率 */
#if (ENCODER_MODE == TIM_ENCODERMODE_TI12)
  #define ENCODER_TOTAL_RESOLUTION             (ENCODER_RESOLUTION * 4)  /* 4倍频后的总分辨率 */
#else
  #define ENCODER_TOTAL_RESOLUTION             (ENCODER_RESOLUTION * 2)  /* 2倍频后的总分辨率 */
#endif


// 计算速度的宏定义
#define OUTPUT_SPEED(param)  ((float)param / ENCODER_TOTAL_RESOLUTION)

void initEncoder(void);

#endif
