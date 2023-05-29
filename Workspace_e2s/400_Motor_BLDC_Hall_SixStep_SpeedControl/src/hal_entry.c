#include "hal_data.h"

FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

#include "led/bsp_led.h"
#include "debug_uart/bsp_debug_uart.h"
#include "key_irq/bsp_key_irq.h"
#include "r_bldc/r_bldc.h"
#include "adc/bsp_adc_vr.h"

/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void)
{
    /* TODO: add your own code here */

	//初始化
	LED_Init();
	initMotor();
	Debug_UART9_Init();
	IRQ_Init();
	adc_Init();
	
	printf("欢迎使用启明6T2开发板\r\n");
	printf("本例程为直流无刷电机六步换向例程，使用驱动接口1进行控制\r\n");
	printf("以下为控制方法：\r\n");
	printf("按下按键1启动或停止无刷电机，按下按键2更换无刷电机旋转方向\r\n");
	printf("旋转电位器控制无刷电机旋转速度，顺时针旋转加速，逆时针旋转则减速\r\n");
	
	while(1)
	{
		/*无刷电机六步换向*/
		Change();
			
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