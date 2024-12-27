#include "hal_data.h"

FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

#include "led/bsp_led.h"
#include "key/key.h"
#include "debug_uart/bsp_debug_uart.h"
#include "motor_controls/bsp_motor_control.h"
#include <motor_controls/bsp_circular_interpolation.h>

// 按键2中断回调函数
void sw2_irq_callback(external_irq_callback_args_t *p_args)
{
    // 防止回调函数中没有使用形参的警告产生
    FSP_PARAMETER_NOT_USED(p_args);

    // 执行第四象限顺时针圆弧插补运动
    Circular_InterPolation(SPR * 5, 0, 0, -SPR * 5, 3200, CW);
    while(circular_para.motionstatus);
}

// 按键3中断回调函数
void sw3_irq_callback(external_irq_callback_args_t *p_args)
{
    // 防止回调函数中没有使用形参的警告产生
    FSP_PARAMETER_NOT_USED(p_args);

    // 执行第三象限逆时针圆弧插补运动
    Circular_InterPolation(0, -SPR * 5, -SPR * 5, 0, 3200, CCW);
    while(circular_para.motionstatus);
}


/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void)
{
    /* TODO: add your own code here */
	LED_Init(); // LED 初始化
	Debug_UART9_Init();//调试串口初始化
	IRQ_Init(); //按键中断初始化
	
	Motor_Control_Init();	   //初始化电机定时器
	
    MOTOR_PRINT("这是一个瑞萨RA，步进电机任意象限双向圆弧插补例程\r\n");

	while(1)
	{}
	
#if BSP_TZ_SECURE_BUILD
    /* Enter non-secure code */
    R_BSP_NonSecureEnter();
#endif
}



/*******************************************************************************************************************//**
 * This function is called at various points during the startup process.  This implementation uses the event that is
 * called right before main() to set up the pins.
 *
 * @param[in]  event    Where at in the start up process the code is currently at
 **********************************************************************************************************************/
void R_BSP_WarmStart (bsp_warm_start_event_t event)
{
    if (BSP_WARM_START_RESET == event)
    {
#if BSP_FEATURE_FLASH_LP_VERSION != 0

        /* Enable reading from data flash. */
        R_FACI_LP->DFLCTL = 1U;

        /* Would normally have to wait tDSTOP(6us) for data flash recovery. Placing the enable here, before clock and
         * C runtime initialization, should negate the need for a delay since the initialization will typically take more than 6us. */
#endif
    }

    if (BSP_WARM_START_POST_C == event)
    {
        /* C runtime environment and system clocks are setup. */

        /* Configure pins. */
        R_IOPORT_Open(&g_ioport_ctrl, g_ioport.p_cfg);
    }
}

#if BSP_TZ_SECURE_BUILD

BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ();

/* Trustzone Secure Projects require at least one nonsecure callable function in order to build (Remove this if it is not required to build). */
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ()
{

}
#endif
