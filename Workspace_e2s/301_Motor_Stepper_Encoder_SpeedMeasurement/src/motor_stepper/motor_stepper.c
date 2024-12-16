#include "motor_stepper.h"
#include "key/key.h"
#include "r_encoder/encoder.h"

/* 定时器初始化函数 */
void initMotor(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /* 初始化 GPT 模块 */
    err = R_GPT_Open(&step_pwm_ctrl, &step_pwm_cfg);
    assert(FSP_SUCCESS == err);
    
    /* 启动 GPT 定时器 */
    err = R_GPT_Start(&step_pwm_ctrl);
    assert(FSP_SUCCESS == err);
}

/*按键1中断回调函数*/
void key1_irq_callback(external_irq_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);

	/*控制步进电机启动*/
	STEP_ENBLED;
	initEncoder();
}


/*按键2中断回调函数*/
void key2_irq_callback(external_irq_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);

	/*控制步进电机停止*/
	STEP_DISENBLED;
	
}


/**
 * @brief 步进电机速度控制
 * @param[in] speed：速度
 * @note 
**/
void setStepSpeed(uint32_t speed)
{
	STEP_DISENBLED;
	
	R_GPT_Stop(&step_pwm_ctrl);
	
	R_GPT_PeriodSet(&step_pwm_ctrl, speed);
	
	R_GPT_Start(&step_pwm_ctrl);
	
	STEP_ENBLED;

}



