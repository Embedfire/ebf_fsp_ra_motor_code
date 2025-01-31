#include "hal_data.h"
#include "led/bsp_led.h"
#include "beep/bsp_beep.h"
#include "key/bsp_key_irq.h"
#include "debug_uart/bsp_debug_uart.h"
#include "stepper/bsp_stepper_init.h"
#include "stepper/bsp_stepper_S_speed.h"
FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

extern Stepper_Typedef Stepper;

/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void)
{
    /* TODO: add your own code here */
    LED_Init();                              // LED 初始化
    Debug_UART9_Init();
    IRQ_Init();
    TIM_PWMOUTPUT_Config();

    printf("这是一个 启明 6T2 串口收发回显例程\r\n");
    printf("打开串口助手发送数据，接收窗口会回显所发送的数据\r\n");

    while(1)
    {
        if(print_flag)
        {
            /*步进电机加速部分*/
            stepper_move_S(1,100,0.1f);
            print_flag=0;
        }
        if( Stepper.status == AVESPEED)
        {
            /*步进电机减速部分*/
            stepper_move_S(100,1,0.1f);
            Stepper.status = DECEL;
        }
    }


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
void R_BSP_WarmStart(bsp_warm_start_event_t event)
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
        R_IOPORT_Open (&g_ioport_ctrl, g_ioport.p_cfg);
    }
}

#if BSP_TZ_SECURE_BUILD

BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ();

/* Trustzone Secure Projects require at least one nonsecure callable function in order to build (Remove this if it is not required to build). */
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ()
{

}
#endif
