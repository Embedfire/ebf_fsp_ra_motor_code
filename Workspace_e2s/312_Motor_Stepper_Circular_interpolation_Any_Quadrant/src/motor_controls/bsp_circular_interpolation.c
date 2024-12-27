/**
  ******************************************************************************
  * @file    bsp_stepper_liner_interpolation.c
  * @author  fire
  * @version V1.0
  * @date    2024-xx-xx
  * @brief   任意象限双向圆弧插补-逐点比较法
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
  * @brief  设置进给方向
  * @param  coord_x
  * @param  coord_y
  * @retval 无
  */
static void Set_Feed_DIR(int32_t coord_x, int32_t coord_y, uint8_t dir)
{
  /* 记录插补运动方向 */
  circular_para.dir_interpo = dir;

  if(dir == CW)
  {
    if(coord_x > 0)/* x正半轴 */
    {
      if(coord_y > 0)/* 第一象限 */
      {
        circular_para.crood_pos = quadrant_1st;
        circular_para.dir_x = CW;
        circular_para.dir_y = CCW;
        circular_para.devi_sign[x_axis] = 1;
        circular_para.devi_sign[y_axis] = -1;
        X_STEP_CW;
        Y_STEP_CCW;
      }
      else/* 第四象限 */
      {
        circular_para.crood_pos = quadrant_4th;
        circular_para.dir_x = CCW;
        circular_para.dir_y = CCW;
        circular_para.devi_sign[x_axis] = -1;
        circular_para.devi_sign[y_axis] = -1;
        X_STEP_CCW;
        Y_STEP_CCW;
      }
    }
    else if(coord_x < 0)/* x负半轴 */
    {
      if(coord_y >= 0)/* 第二象限 */
      {
        circular_para.crood_pos = quadrant_2nd;
        circular_para.dir_x = CW;
        circular_para.dir_y = CW;
        circular_para.devi_sign[x_axis] = 1;
        circular_para.devi_sign[y_axis] = 1;
        X_STEP_CW;
        Y_STEP_CW;
      }
      else/* 第三象限 */
      {
        circular_para.crood_pos = quadrant_3rd;
        circular_para.dir_x = CCW;
        circular_para.dir_y = CW;
        circular_para.devi_sign[x_axis] = -1;
        circular_para.devi_sign[y_axis] = 1;
        X_STEP_CCW;
        Y_STEP_CW;
      }
    }
    else if(coord_x == 0)/* x=0，当前点在Y轴上 */
    {
      if(coord_y > 0)/* 第一象限 */
      {
        circular_para.crood_pos = quadrant_1st;
        circular_para.dir_x = CW;
        circular_para.dir_y = CCW;
        circular_para.devi_sign[x_axis] = 1;
        circular_para.devi_sign[y_axis] = -1;
        X_STEP_CW;
        Y_STEP_CCW;
      }
      else if(coord_y < 0)/* 第三象限 */
      {
        circular_para.crood_pos = quadrant_3rd;
        circular_para.dir_x = CCW;
        circular_para.dir_y = CW;
        circular_para.devi_sign[x_axis] = -1;
        circular_para.devi_sign[y_axis] = 1;
        X_STEP_CCW;
        Y_STEP_CW;
      }
    }
  }
  else
  {
    if(coord_x > 0)/* x正半轴 */
    {
      if(coord_y >= 0)/* 第一象限 */
      {
        circular_para.crood_pos = quadrant_1st;
        circular_para.dir_x = CCW;
        circular_para.dir_y = CW;
        circular_para.devi_sign[x_axis] = -1;
        circular_para.devi_sign[y_axis] = 1;
        X_STEP_CCW;
        Y_STEP_CW;
      }
      else/* 第四象限 */
      {
        circular_para.crood_pos = quadrant_4th;
        circular_para.dir_x = CW;
        circular_para.dir_y = CW;
        circular_para.devi_sign[x_axis] = 1;
        circular_para.devi_sign[y_axis] = 1;
        X_STEP_CW;
        Y_STEP_CW;
      }
    }
    else if(coord_x < 0)/* x负半轴 */
    {
      if(coord_y > 0)/* 第二象限 */
      {
        circular_para.crood_pos = quadrant_2nd;
        circular_para.dir_x = CCW;
        circular_para.dir_y = CCW;
        circular_para.devi_sign[x_axis] = -1;
        circular_para.devi_sign[y_axis] = -1;
        X_STEP_CCW;
        Y_STEP_CCW;
      }
      else/* 第三象限 */
      {
        circular_para.crood_pos = quadrant_3rd;
        circular_para.dir_x = CW;
        circular_para.dir_y = CCW;
        circular_para.devi_sign[x_axis] = 1;
        circular_para.devi_sign[y_axis] = -1;
        X_STEP_CW;
        Y_STEP_CCW;
      }
    }
    else if(coord_x == 0)/* x=0，当前点在Y轴上 */
    {
      if(coord_y > 0)/* 第二象限 */
      {
        circular_para.crood_pos = quadrant_2nd;
        circular_para.dir_x = CCW;
        circular_para.dir_y = CCW;
        circular_para.devi_sign[x_axis] = -1;
        circular_para.devi_sign[y_axis] = -1;
        X_STEP_CCW;
        Y_STEP_CCW;
      }
      else if(coord_y < 0)/* 第四象限 */
      {
        circular_para.crood_pos = quadrant_4th;
        circular_para.dir_x = CW;
        circular_para.dir_y = CW;
        circular_para.devi_sign[x_axis] = 1;
        circular_para.devi_sign[y_axis] = 1;
        X_STEP_CW;
        Y_STEP_CW;
      }
    }
  }
}

/**
  * @brief  任意象限顺圆插补运动
  * @param  start_x：圆弧起点坐标X
  * @param  start_y：圆弧起点坐标Y
  * @param  stop_x：圆弧终点坐标X
  * @param  stop_y：圆弧终点坐标Y
  * @param  speed：进给速度
  * @param  dir：进给方向
  * @retval 无
  */
void Circular_InterPolation(int32_t start_x, int32_t start_y, int32_t stop_x, int32_t stop_y, uint16_t speed, uint8_t dir)
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
  circular_para.startpoint[x_axis] = start_x;
  circular_para.startpoint[y_axis] = start_y;
  /* 终点坐标 */
  circular_para.endpoint_x = (int32_t)stop_x;
  circular_para.endpoint_y = (int32_t)stop_y;
  /* 所需脉冲数是从起点到终点的脉冲数之和 */
  circular_para.endpoint_pulse = (uint32_t)abs(stop_x - start_x) + (uint32_t)abs(stop_y - start_y);

  /* 根据坐标确定插补方向和X、Y运动方向 */
  Set_Feed_DIR(circular_para.startpoint[x_axis], circular_para.startpoint[y_axis], dir);

  /* 起点坐标x=0，说明起点在y轴上，直接向x轴进给可减小误差 */
  if(circular_para.startpoint[x_axis] == 0)
  {
    /* 偏差方程：F = F ± 2 * x + 1*/
    circular_para.active_axis = x_axis;
    circular_para.deviation += 2 * circular_para.devi_sign[x_axis]
                                 * circular_para.startpoint[x_axis] + 1;
  }
  else
  {
    /* 偏差方程：F = F ± 2 * y + 1*/
    circular_para.active_axis = y_axis;
    circular_para.deviation += 2 * circular_para.devi_sign[y_axis]
                                 * circular_para.startpoint[y_axis] + 1;
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
          case CCW: circular_para.startpoint[x_axis]--; break;
          case CW:  circular_para.startpoint[x_axis]++; break;
        }
        break;
      case y_axis:
        switch(circular_para.dir_y)
        {
          case CCW: circular_para.startpoint[y_axis]--; break;
          case CW:  circular_para.startpoint[y_axis]++; break;
      }
      break;
  }

  /* 根据上一次进给的偏差，判断新的进给活动轴 */
  if(circular_para.deviation >= 0)
  {
    switch(circular_para.dir_interpo)
    {
      case CW:/* 双向 */
        switch(circular_para.crood_pos)
        {
          case quadrant_1st:
          case quadrant_3rd:
            circular_para.active_axis = y_axis;
            break;
          case quadrant_2nd:
          case quadrant_4th:
            circular_para.active_axis = x_axis;
            break;
        }
        break;
      case CCW:/* 逆时针 */
        switch(circular_para.crood_pos)
        {
          case quadrant_1st:
          case quadrant_3rd:
            circular_para.active_axis = x_axis;
            break;
          case quadrant_2nd:
          case quadrant_4th:
            circular_para.active_axis = y_axis;
            break;
        }
        break;
    }
  }
  else /* 偏差小于0，向圆外进给 */
  {
    switch(circular_para.dir_interpo)
    {
      case CW:/* 双向 */
        switch(circular_para.crood_pos)
        {
          case quadrant_1st:
          case quadrant_3rd:
            circular_para.active_axis = x_axis;
            break;
          case quadrant_2nd:
          case quadrant_4th:
            circular_para.active_axis = y_axis;
            break;
        }
        break;
      case CCW:/* 逆时针 */
        switch(circular_para.crood_pos)
        {
          case quadrant_1st:
          case quadrant_3rd:
            circular_para.active_axis = y_axis;
            break;
          case quadrant_2nd:
          case quadrant_4th:
            circular_para.active_axis = x_axis;
            break;
        }
        break;
    }
  }

  /* 根据插补运动方向和进给方向计算出新的偏差 */
  circular_para.deviation += 2 * circular_para.devi_sign[circular_para.active_axis]
                               * circular_para.startpoint[circular_para.active_axis] + 1;

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
