#ifndef __BSP_PID_H
#define __BSP_PID_H

#include "hal_data.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*pid*/
typedef struct
{
  float target_val;       //目标值
    float actual_val;     //实际值
    float err;            //定义当前偏差值
    float err_next;       //定义下一个偏差值
    float err_last;       //定义最后一个偏差值
    float Kp, Ki, Kd;     //定义比例、积分、微分系数
}_pid;

void PID_param_init(void);
void set_pid_target(int temp_val);
float get_pid_target(void);
void set_p_i_d(float p, float i, float d);
int PID_realize(void);
void time_period_fun(void);

#endif
