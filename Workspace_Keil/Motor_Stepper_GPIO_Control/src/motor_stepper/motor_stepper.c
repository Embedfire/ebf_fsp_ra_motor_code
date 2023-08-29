#include "motor_stepper.h"
#include "key/key.h"

/* ��ʱ����ʼ������ */
void initMotor(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /* ��ʼ�� GPT ģ�� */
    err = R_GPT_Open(&step_pwm_ctrl, &step_pwm_cfg);
    assert(FSP_SUCCESS == err);
    
    /* ���� GPT ��ʱ�� */
    err = R_GPT_Start(&step_pwm_ctrl);
    assert(FSP_SUCCESS == err);
}

/*����1�жϻص�����*/
void key1_irq_callback(external_irq_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);

	/*���Ʋ����������*/
	STEP_ENBLED;
}


/*����2�жϻص�����*/
void key2_irq_callback(external_irq_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);

	/*���Ʋ������ֹͣ*/
	STEP_DISENBLED;
	
}


/**
 * @brief ��������ٶȿ���
 * @param[in] speed���ٶ�
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



