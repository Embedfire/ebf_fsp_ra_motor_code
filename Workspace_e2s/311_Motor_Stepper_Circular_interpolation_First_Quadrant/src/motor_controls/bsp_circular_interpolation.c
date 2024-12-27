/**
  ******************************************************************************
  * @file    bsp_stepper_liner_interpolation.c
  * @author  fire
  * @version V1.0
  * @date    2024-xx-xx
  * @brief   第一象限双向圆弧插补-逐点比较法
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 启明 6T2 开发板
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://yehuosm.tmall.com
  *
  ******************************************************************************
  */
#include <motor_controls/bsp_circular_interpolation.h>

CircularInterpolation_TypeDef circular_para = {0};

/**
  * @brief  第一象限顺圆插补运动
  * @param  start_x：圆弧起点相对于圆心的坐标X，增量
  * @param  inc_y：终点坐标Y的增量
  * @param  speed：进给速度
  * @retval 无
  */
void Circular_InterPolation_CW(int32_t start_x, int32_t start_y, int32_t stop_x, int32_t stop_y, uint32_t speed)
{
  /* 判断当前是否正在做插补运动 */
  if(circular_para.motionstatus != 0)
    return;

  /* 检查起点、终点坐标是否在同一个圆上 */
  if(((start_x * start_x) + (start_y * start_y)) != ((stop_x * stop_x) + (stop_y * stop_y)))
    return;

  /* 偏差清零 */
  circular_para.deviation = 0;

  /* 起点坐标 */
  circular_para.startpoint_x = start_x;
  circular_para.startpoint_y = start_y;
  /* 终点坐标 */
  circular_para.endpoint_x = (uint32_t)stop_x;
  circular_para.endpoint_y = (uint32_t)stop_y;
  /* 所需脉冲数是从起点到终点的脉冲数之和 */
  circular_para.endpoint_pulse = (uint32_t)abs(stop_x - start_x) + (uint32_t)abs(stop_y - start_y);

  /* 第一象限正圆，x轴正转，y轴逆转 */
  circular_para.dir_x = CW;
  circular_para.dir_y = CCW;
  X_STEP_CW;    // X 轴顺时针方向
  Y_STEP_CCW;   // Y 轴逆时针方向

  /* 起点坐标x=0，说明起点在y轴上，直接向x轴进给可减小误差 */
  if(circular_para.startpoint_x == 0)
  {
    /* 第一步活动轴为x轴 */
    circular_para.active_axis = x_axis;
    /* 计算偏差 */
    circular_para.deviation += (2 * circular_para.startpoint_x + 1);
  }
  else
  {
    /* 第一步活动轴为Y轴 */
    circular_para.active_axis = y_axis;
    /* 计算偏差 */
    circular_para.deviation -= (2 * circular_para.startpoint_y + 1);
  }

  // 设置电机的速度
  Motor_Control_SetSpeed(speed, speed);

  if(circular_para.active_axis == x_axis)
      // 启动 X 轴电机控制
      X_Motor_Control_Start();
  else
      // 启动 Y 轴电机控制
      Y_Motor_Control_Start();

  circular_para.motionstatus = 1;
}

/**
  * @brief  第一象限逆圆插补运动
  * @param  start_x：圆弧起点相对于圆心的坐标X，增量
  * @param  inc_y：终点坐标Y的增量
  * @param  speed：进给速度
  * @retval 无
  */
void Circular_InterPolation_CCW(int32_t start_x, int32_t start_y, int32_t stop_x, int32_t stop_y, uint32_t speed)
{
  /* 判断当前是否正在做插补运动 */
  if(circular_para.motionstatus != 0)
    return;

  /* 检查起点、终点坐标是否在同一个圆上 */
  if(((start_x * start_x) + (start_y * start_y)) != ((stop_x * stop_x) + (stop_y * stop_y)))
    return;

  /* 偏差清零 */
  circular_para.deviation = 0;

  /* 起点坐标 */
  circular_para.startpoint_x = start_x;
  circular_para.startpoint_y = start_y;
  /* 终点坐标 */
  circular_para.endpoint_x = (uint32_t)stop_x;
  circular_para.endpoint_y = (uint32_t)stop_y;
  /* 所需脉冲数是从起点到终点的脉冲数之和 */
  circular_para.endpoint_pulse = (uint32_t)abs(stop_x - start_x) + (uint32_t)abs(stop_y - start_y);

  /* 第一象限逆圆，x轴逆转，y轴正转 */
  circular_para.dir_x = CCW;
  circular_para.dir_y = CW;
  X_STEP_CCW;    // X 轴逆时针方向
  Y_STEP_CW;   // Y 轴顺时针方向

  /* 起点坐标y=0，说明起点在x轴上，直接向y轴进给可减小误差 */
  if(circular_para.startpoint_y == 0)
  {
    /* 第一步活动轴为Y轴 */
    circular_para.active_axis = y_axis;
    /* 计算偏差 */
    circular_para.deviation += (2 * circular_para.startpoint_y + 1);
  }
  else
  {
    /* 第一步活动轴为X轴 */
    circular_para.active_axis = x_axis;
    /* 计算偏差 */
    circular_para.deviation -= (2 * circular_para.startpoint_x + 1);
  }

  // 设置电机的速度
  Motor_Control_SetSpeed(speed, speed);

  if(circular_para.active_axis == x_axis)
      // 启动 X 轴电机控制
      X_Motor_Control_Start();
  else
      // 启动 Y 轴电机控制
      Y_Motor_Control_Start();

  circular_para.motionstatus = 1;
}

/**
  * @brief  定时器比较中断回调函数
  * @param  htim：定时器句柄指针
	*	@note   无
  * @retval 无
  */
void x_y_step_callback(timer_callback_args_t *p_args)
{
  if(p_args -> event == TIMER_EVENT_COMPARE_B){

  uint32_t last_axis = 0;

  /* 记录上一步的进给活动轴 */
  last_axis = circular_para.active_axis;

  /* 根据进给方向刷新坐标 */
  switch(last_axis)
  {
    case x_axis:
      switch(circular_para.dir_x)
      {
        case CCW: circular_para.startpoint_x--; break;
        case CW:  circular_para.startpoint_x++; break;
      }
      break;
    case y_axis:
      switch(circular_para.dir_y)
      {
        case CCW: circular_para.startpoint_y--; break;
        case CW:  circular_para.startpoint_y++; break;
      }
      break;
  }

  /* 根据上一步的偏差，判断的进给方向，并计算下一步的偏差 */
  if(circular_para.deviation >= 0)
  {
    /* 偏差>=0，在圆弧外侧，应向圆内进给，计算偏差 */
    circular_para.active_axis = y_axis;
    circular_para.deviation -= (2 * circular_para.startpoint_y + 1);
  }
  else
  {
    /* 偏差<0，在圆弧内侧，应向圆外进给，计算偏差 */
    circular_para.active_axis = x_axis;
    circular_para.deviation += (2 * circular_para.startpoint_x + 1);
  }


  /* 下一步的活动轴与上一步的不一致时，需要换轴 */
  if(last_axis != circular_para.active_axis)
  {
      // 判断活动轴是否为 X 轴
      if (circular_para.active_axis == x_axis)
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
  circular_para.endpoint_pulse--;

  /* 判断是否完成插补 */
  if(circular_para.endpoint_pulse == 0)
  {
    /* 关闭定时器 */
      X_Motor_Control_Stop();
      Y_Motor_Control_Stop();

      // 重置插补参数中的运动状态
      circular_para.motionstatus = 0;

  }
 }
}
