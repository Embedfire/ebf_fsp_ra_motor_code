#ifndef __BSP_LINEAR_INTERPOLATION_H
#define __BSP_LINEAR_INTERPOLATION_H

#include "hal_data.h"
#include "bsp_motor_control.h"
#include <stdlib.h>

/* 坐标轴枚举 */
typedef enum{
  x_axis = 0U,
  y_axis
}Axis_TypeDef;

/* 坐标轴象限枚举 */
typedef enum{
  quadrant_1st = 0U,
  quadrant_2nd,
  quadrant_3rd,
  quadrant_4th
}Quadrant_TypeDef;

/* 方向枚举 */
typedef enum{
    CW = 0U,  // 正转 (顺时针 Clockwise)
    CCW       // 反转 (逆时针 Counter-Clockwise)
} Direction_TypeDef;


/* 圆弧插补参数结构体 */
typedef struct{
  __IO int32_t startpoint[2];        //起点坐标X、Y
  __IO int32_t endpoint_x;           //终点坐标X
  __IO int32_t endpoint_y;           //终点坐标Y
  __IO uint32_t endpoint_pulse;      //到达终点位置需要的脉冲数
  __IO uint32_t active_axis;         //当前运动的轴
  __IO int32_t deviation;            //偏差参数F
  __IO int8_t devi_sign[2];          //偏差方程的运算符号，正负
  __IO uint8_t motionstatus : 1;     //插补运动状态
  __IO uint8_t dir_x : 1;            //X轴运动方向
  __IO uint8_t dir_y : 1;            //Y轴运动方向
  __IO uint8_t dir_interpo : 1;      //插补整体运动方向
  __IO uint8_t crood_pos : 2;        //起点坐标所在的象限
}CircularInterpolation_TypeDef;


extern CircularInterpolation_TypeDef circular_para;

void Circular_InterPolation(int32_t start_x, int32_t start_y, int32_t stop_x, int32_t stop_y, uint16_t speed, uint8_t dir);
#endif /* __BSP_STEP_LINEAR_INTERPOLATION_H */
