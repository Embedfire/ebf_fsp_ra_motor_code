#include "motor_dc_brush.h"


/* ��ʱ����ʼ������ */
void initMotor(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /* ��ʼ�� GPT ģ�� */
    err = R_GPT_Open(&brush_pwm1_ctrl, &brush_pwm1_cfg);
    assert(FSP_SUCCESS == err);
    
    /* ���� PWM1 ��ʱ�� */
    err = R_GPT_Start(&brush_pwm1_ctrl);
    assert(FSP_SUCCESS == err);
	
	
    /* ��ʼ�� GPT ģ�� */
    err = R_GPT_Open(&brush_pwm2_ctrl, &brush_pwm2_cfg);
    assert(FSP_SUCCESS == err);
    
    /* ���� PWM2 ��ʱ�� */
    //err = R_GPT_Start(&brush_pwm2_ctrl);
    //assert(FSP_SUCCESS == err);
}

/*����1�жϻص�����*/
void sw2_callback(external_irq_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);

	/*������ˢ�������*/
	BRUSH_ENBLED;

}


/*����2�жϻص�����*/
void sw3_callback(external_irq_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);

	/*������ˢ���ֹͣ*/
	BRUSH_DISENBLED;

}

/**
 * @brief ֱ����ˢ����ٶ�
 * @param[in] speed:����ٶȣ���Χ��0~100%
 */
void setBrushSpeed(uint8_t speed)
{
    timer_info_t info;
    uint32_t current_period_counts;
	uint32_t duty_cycle_counts;
	
    if (speed > 100)
        speed = 100; //�����ٶȷ�Χ��0~100

    if (speed < 0)
        speed = 0; //�����ٶȷ�Χ��0~100
	
    /* ���GPT����Ϣ */
    R_GPT_InfoGet(&brush_pwm1_ctrl, &info);

    /* ��ü�ʱ��һ��������Ҫ�ļ������� */
    current_period_counts = info.period_counts;

    /* ����ռ�ձȺ�һ�����ڵļ�����������GTCCR�Ĵ�����ֵ */
    duty_cycle_counts = (uint32_t)(((uint64_t) current_period_counts * speed) / 100);

    /* ������FSP�⺯�������ٶ� */
    R_GPT_DutyCycleSet(&brush_pwm1_ctrl, duty_cycle_counts, GPT_IO_PIN_GTIOCA);
	
    R_GPT_DutyCycleSet(&brush_pwm2_ctrl, duty_cycle_counts, GPT_IO_PIN_GTIOCA);
	
}


