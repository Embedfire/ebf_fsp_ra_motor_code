#ifndef __BSP_PID_H
#define __BSP_PID_H
#include "hal_data.h"
#include "debug_uart/bsp_debug_uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

typedef struct
{
    float target_val;               //目标值
    float actual_val;               //实际值
    float err;                      //定义偏差值
    float err_last;                 //定义上一个偏差值
    float Kp,Ki,Kd;                 //定义比例、积分、微分系数
    float integral;                 //定义积分值
}_pid;

extern _pid pid_location;
extern _pid pid_speed;
extern _pid pid_curr;

extern void PID_param_init(void);
extern void set_pid_target(_pid *pid, float temp_val);
extern float get_pid_target(_pid *pid);
extern void set_p_i_d(_pid *pid, float p, float i, float d);
extern float location_pid_realize(_pid *pid, float actual_val);
extern float speed_pid_realize(_pid *pid, float actual_val);
extern float curr_pid_realize(_pid *pid, float actual_val);

#endif
