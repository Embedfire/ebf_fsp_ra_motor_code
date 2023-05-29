#include "bsp_key_irq.h"
#include "led/bsp_led.h"
#include "r_bldc/r_bldc.h"

extern _Bool Bldc_Direction;

void IRQ_Init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /*开启外部中断*/
    err = R_ICU_ExternalIrqOpen(&sw2_irq_ctrl, &sw2_irq_cfg);
    assert(FSP_SUCCESS == err);
    err = R_ICU_ExternalIrqOpen(&sw3_irq_ctrl, &sw3_irq_cfg);
    assert(FSP_SUCCESS == err);
    
    /*使能外部中断*/
    err = R_ICU_ExternalIrqEnable(&sw2_irq_ctrl);
    assert(FSP_SUCCESS == err);
    err = R_ICU_ExternalIrqEnable(&sw3_irq_ctrl);
    assert(FSP_SUCCESS == err);
    
}


/*按键1中断回调函数*/
void sw2_irq_callback(external_irq_callback_args_t *p_args)
{
	 /*防止回调函数中没有使用形参的警告产生*/
     FSP_PARAMETER_NOT_USED(p_args);
    
	 MOTOR_STATE_TOGGLE;
	 Bldc_Direction = !Bldc_Direction;

    
}

/*按键2中断回调函数*/
void sw3_irq_callback(external_irq_callback_args_t *p_args)
{
	/*防止回调函数中没有使用形参的警告产生*/
    FSP_PARAMETER_NOT_USED(p_args);

	Bldc_Direction = !Bldc_Direction;
    
}