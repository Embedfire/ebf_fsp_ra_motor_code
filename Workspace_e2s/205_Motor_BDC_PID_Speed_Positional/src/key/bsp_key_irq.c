#include "bsp_key_irq.h"
#include "led/bsp_led.h"
//
//void IRQ_Init(void)
//{
//    fsp_err_t err = FSP_SUCCESS;
//    //开启外部中断
//    err = R_ICU_ExternalIrqOpen(&sw2_irq_ctrl, &sw2_irq_cfg);
//    assert(FSP_SUCCESS == err);
//    //err = R_ICU_ExternalIrqOpen(&sw3_irq_ctrl, &sw3_irq_cfg);
//    //assert(FSP_SUCCESS == err);
//
//    //使能外部中断
//    err = R_ICU_ExternalIrqEnable(&sw2_irq_ctrl);
//    assert(FSP_SUCCESS == err);
//    //err = R_ICU_ExternalIrqEnable(&sw3_irq_ctrl);
//    //assert(FSP_SUCCESS == err);
//
//}



/*
//按键3中断回调函数
void sw3_irq_callback(external_irq_callback_args_t *p_args)
{
    //防止回调函数中没有使用形参的警告产生
    FSP_PARAMETER_NOT_USED(p_args);

    LED1_OFF; // LED1灭
    LED4_OFF; // LED4灭

}
*/
