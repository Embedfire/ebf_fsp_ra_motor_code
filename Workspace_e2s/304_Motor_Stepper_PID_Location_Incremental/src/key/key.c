#include "key.h"
#include "led/bsp_led.h"

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


