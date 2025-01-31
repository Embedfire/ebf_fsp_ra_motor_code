#ifndef __BSP_STEPPER_T_SPEED_H
#define __BSP_STEPPER_T_SPEED_H

#include "hal_data.h"
#include "stepper/bsp_stepper_init.h"
#include "math.h"

//梯形加减速相关变量
typedef struct {
  //电机运行状态
  unsigned char run_state : 3;
  //电机运行方向
  unsigned char dir : 1;
  //下一个脉冲延时周期，启动时为加速度速率
  unsigned int step_delay;
  //开始减速的位置
  unsigned int decel_start;
  //减速距离
  signed int decel_val;
  //最小延时（即最大速度）
  signed int min_delay;
  //加速或者减速计数器
  signed int accel_count;
} speedRampData;


//系统状态
struct GLOBAL_FLAGS {
  //当步进电机正在运行时，值为1
  unsigned int running:1;
  //当串口接收到数据时，值为1
  unsigned int cmd:1;
  //当驱动器正常输出时,值为1
  unsigned int out_ena:1;
};



#define FALSE             0
#define TRUE              1
#define CW                0 // 顺时针
#define CCW               1 // 逆时针

/*电机速度决策中的四个状态*/
#define STOP              0 // 停止状态
#define ACCEL             1 // 加速状态
#define DECEL             2 // 减速状态
#define RUN               3 // 匀速状态

/*频率相关参数*/
#define T1_FREQ           (8000000) // 频率ft值


/*电机单圈参数*/
#define STEP_ANGLE              1.8                                 //步进电机的步距角 单位：度
#define FSPR              (360.0/1.8)         //步进电机的一圈所需脉冲数

#define MICRO_STEP        32                        //细分器细分数
#define SPR               (FSPR*MICRO_STEP)   //细分后一圈所需脉冲数

/*数学常数,用于简化计算*/

#define ALPHA             ((float)(2*3.14159/SPR))       // α= 2*pi/spr
#define A_T_x10           ((float)(10*ALPHA*T1_FREQ))
#define T1_FREQ_148       ((float)((T1_FREQ*0.676)/10)) // 0.676为误差修正值
#define A_SQ              ((float)(2*100000*ALPHA)) 
#define A_x200            ((float)(200*ALPHA))

extern void speed_decision(void);
extern int stepPosition;
extern struct GLOBAL_FLAGS status;

#endif
