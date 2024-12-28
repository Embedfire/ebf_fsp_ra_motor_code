#include "stepper/bsp_stepper_init.h"
#include "stepper/bsp_stepper_S_speed.h"
#include "hal_data.h"
#include "math.h"
#include <math.h>


void TIM_PWMOUTPUT_Config(void)
{
    R_GPT_Open(&g_timer4_ctrl,&g_timer4_cfg);

    R_GPT_Enable(&g_timer4_ctrl);
}

/**
  * @brief  引脚初始化
  * @retval 无
  */
void stepper_Init()
{
	/*定时器PWM输出配置*/
	TIM_PWMOUTPUT_Config();
}

/**
  * @brief  定时器中断服务函数
  * @retval 无
  */
void gpt4_callback(timer_callback_args_t *p_args)
{
	/*速度状态决策*/
	speed_decision();
}
