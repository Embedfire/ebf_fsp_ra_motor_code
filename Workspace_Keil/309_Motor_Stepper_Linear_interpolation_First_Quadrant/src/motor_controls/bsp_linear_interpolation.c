/**
  ******************************************************************************
  * @file    bsp_stepper_liner_interpolation.c
  * @author  fire
  * @version V1.0
  * @date    2024-xx-xx
  * @brief   第一象限直线插补-逐点比较法
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 启明 6T2 开发板
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://yehuosm.tmall.com
  *
  ******************************************************************************
  */
#include "bsp_linear_interpolation.h"

Axis_TypeDef axis;
LinearInterpolation_TypeDef interpolation_para = {0};

/**
  * @brief  直线增量插补运动
  * @param  inc_x：终点坐标X的增量
  * @param  inc_y：终点坐标Y的增量
  * @param  speed：进给速度
  * @retval 无
  */
void InterPolation_Move(uint32_t inc_x, uint32_t inc_y, uint32_t speed)
{
  /* 偏差清零 */
  interpolation_para.deviation = 0;
  
  /* 设置终点坐标 */
  interpolation_para.endpoint_x = inc_x;
  interpolation_para.endpoint_y = inc_y;
  /* 所需脉冲数为X、Y坐标增量之和 */
  interpolation_para.endpoint_pulse = inc_x + inc_y;
  
  /* 第一步进给的活动轴为X轴 */
  interpolation_para.active_axis = x_axis;

  /* 计算偏差 */
  interpolation_para.deviation -= (int32_t)interpolation_para.endpoint_y;

  // 设置电机的速度
  Motor_Control_SetSpeed(speed, speed);

  if(interpolation_para.active_axis == x_axis)
      // 启动 X 轴电机控制
      X_Motor_Control_Start();
  else
      // 启动 Y 轴电机控制
      Y_Motor_Control_Start();
}

/**
  * @brief  定时器比较中断回调函数
  * @param  htim：定时器句柄指针
	*	@note   无
  * @retval 无
  */
void x_y_step_callback(timer_callback_args_t *p_args)
{
  if(p_args -> event == TIMER_EVENT_CYCLE_END){

  uint32_t last_axis = 0;

  /* 记录上一步的进给活动轴 */
  last_axis = interpolation_para.active_axis;

  /* 根据上一步的偏差，判断的进给方向，并计算下一步的偏差 */
  if(interpolation_para.deviation >= 0)
  {
    /* 偏差>0，在直线上方，进给X轴，计算偏差 */
    interpolation_para.active_axis = x_axis;
    interpolation_para.deviation -= (int32_t)interpolation_para.endpoint_y;
  }
  else
  {
    /* 偏差<0，在直线下方，进给Y轴，计算偏差 */
    interpolation_para.active_axis = y_axis;
    interpolation_para.deviation += (int32_t)interpolation_para.endpoint_x;
  }

  /* 下一步的活动轴与上一步的不一致时，需要换轴 */
  if(last_axis != interpolation_para.active_axis)
  {
      // 判断活动轴是否为 X 轴
      if (interpolation_para.active_axis == x_axis)
      {
          // 启动 X 轴电机，停止 Y 轴电机
          X_Motor_Control_Start();
          Y_Motor_Control_Stop();
      }
      else
      {
          // 启动 Y 轴电机，停止 X 轴电机
          Y_Motor_Control_Start();
          X_Motor_Control_Stop();
      }
  }
  /* 进给总步数减1 */
  interpolation_para.endpoint_pulse--;

  /* 判断是否完成插补 */
  if(interpolation_para.endpoint_pulse == 0)
  {
    /* 关闭定时器 */
      X_Motor_Control_Stop();
      Y_Motor_Control_Stop();
  }
 }
}
