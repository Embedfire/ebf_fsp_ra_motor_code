#include "pid/bsp_pid.h"
#include "gpt/bsp_gpt_timing.h"
#include "protocol/protocol.h"

//定义全局变量

_pid pid;

/**
  * @brief  PID参数初始化
	*	@note 	无
  * @retval 无
  */
void PID_param_init()
{
		/* 初始化参数 */
        pid.target_val=50;
        pid.actual_val=0.0;

		pid.err = 0.0;
		pid.err_last = 0.0;
		pid.err_next = 0.0;
		
        pid.Kp = 6;
        pid.Ki = 2.5;
        pid.Kd = 0;

		float pid_temp[3] = {pid.Kp, pid.Ki, pid.Kd};
		set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);     // 给通道 1 发送 P I D 值

}

/**
  * @brief  设置目标值
  * @param  val		目标值
	*	@note 	无
  * @retval 无
  */
void set_pid_target(float temp_val)
{
  pid.target_val = temp_val;    // 设置当前的目标值
}

/**
  * @brief  获取目标值
  * @param  无
	*	@note 	无
  * @retval 目标值
  */
float get_pid_target(void)
{
  return pid.target_val;    // 设置当前的目标值
}

/**
  * @brief  设置比例、积分、微分系数
  * @param  p：比例系数 P
  * @param  i：积分系数 i
  * @param  d：微分系数 d
	*	@note 	无
  * @retval 无
  */
void set_p_i_d(float p, float i, float d)
{
  	    pid.Kp = p;    // 设置比例系数 P
		pid.Ki = i;    // 设置积分系数 I
		pid.Kd = d;    // 设置微分系数 D
}

/**
  * @brief  PID算法实现
  * @param  actual_val:实际值
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
float PID_realize(float actual_val)
{
	/*计算目标值与实际值的误差*/
    pid.err=pid.target_val-actual_val;

	/*PID算法实现*/
	pid.actual_val += pid.Kp*(pid.err - pid.err_next) 
                   + pid.Ki*pid.err
                   + pid.Kd*(pid.err - 2 * pid.err_next + pid.err_last);
	
	/* 限幅输出,防止深度饱和 */
	if(pid.actual_val > CLK_GPT_COUNTS) pid.actual_val = CLK_GPT_COUNTS;
	else if(pid.actual_val <= 0) pid.actual_val = 0;
	
	/*传递误差*/
	pid.err_last = pid.err_next;
	pid.err_next = pid.err;
	/*返回当前实际值*/
	return pid.actual_val;
}
