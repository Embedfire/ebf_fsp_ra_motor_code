#include "motor_dc_brush.h"

uint8_t motor_speed = 0;
DC_MOTOR_S dc_motor_s;


/* 定时器初始化函数 */
void initMotor(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /* 初始化 GPT 模块 */
    err = R_GPT_Open(&brush_pwm1_ctrl, &brush_pwm1_cfg);
    assert(FSP_SUCCESS == err);
    
    /* 启动 PWM1 定时器 */
    err = R_GPT_Start(&brush_pwm1_ctrl);
    assert(FSP_SUCCESS == err);
	
	
    /* 初始化 GPT 模块 */
    err = R_GPT_Open(&brush_pwm2_ctrl, &brush_pwm2_cfg);
    assert(FSP_SUCCESS == err);
    
    dc_motor_s.motor_en = 0;
	dc_motor_s.direction = 0;
	dc_motor_s.speed = 0;
	dc_motor_s.now_speed = 0;
	
	
	Set_Brush_Speed(dc_motor_s.speed);
}

/*按键1中断回调函数*/
void sw2_callback(external_irq_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);


	/*控制有刷电机启动*/
	dc_motor_s.motor_en = 1;
	BRUSH_ENBLED;

}


/*按键2中断回调函数*/
void sw3_callback(external_irq_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
	
	dc_motor_s.motor_en = 0;
	BRUSH_DISENBLED;

}
void Set_Brush_Direction(bool direction_val)
{
	if(direction_val == 0)
	{
		BRUSH_CW;
	}
	else
	{
		BRUSH_CCW;
	}
}
void Speed_Up(void)
{
	if(dc_motor_s.speed <= 90)
		dc_motor_s.speed += 10;
}
void Speed_Dowm(void)
{
	if(dc_motor_s.speed >= 10)
		dc_motor_s.speed -= 10;
}
/**
 * @brief 直流有刷电机速度
 * @param[in] speed:电机速度，范围：0~100%
 */
void Set_Brush_Speed(uint8_t speed)
{
    timer_info_t info;
    uint32_t current_period_counts;
	uint32_t duty_cycle_counts;
	
    if (speed > 100)
        speed = 100; //限制速度范围：0~100

    if (speed < 0)
        speed = 0; //限制速度范围：0~100
	
    /* 获得GPT的信息 */
    R_GPT_InfoGet(&brush_pwm1_ctrl, &info);

    /* 获得计时器一个周期需要的计数次数 */
    current_period_counts = info.period_counts;

    /* 根据占空比和一个周期的计数次数计算GTCCR寄存器的值 */
    duty_cycle_counts = (uint32_t)(((uint64_t) current_period_counts * speed) / 100);

    /* 最后调用FSP库函数设置速度 */
    R_GPT_DutyCycleSet(&brush_pwm1_ctrl, duty_cycle_counts, GPT_IO_PIN_GTIOCA);
	
    R_GPT_DutyCycleSet(&brush_pwm2_ctrl, duty_cycle_counts, GPT_IO_PIN_GTIOCA);
	
}


