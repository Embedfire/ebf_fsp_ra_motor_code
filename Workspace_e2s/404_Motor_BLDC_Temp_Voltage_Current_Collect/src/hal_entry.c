#include "hal_data.h"

FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

#include "led/bsp_led.h"
#include "debug_uart/bsp_debug_uart.h"
#include "motor_gpt/bsp_motor_gpt.h"
#include "motor_control/bsp_motor_control.h"
#include "adc/bsp_adc_vr.h"
#include "motor_gpt/bsp_gpt_timing.h"

extern volatile bool uart_recv_motor_enable;
extern volatile bool uart_recv_motor_disenable;
extern volatile bool uart_recv_motor_speed_up;
extern volatile bool uart_recv_motor_speed_down;
extern volatile bool uart_recv_motor_reverse;


__IO uint16_t ChannelPulse = 10;


/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void)
{
    /* TODO: add your own code here */
    /* LED初始化 */
	LED_Init();

	/* 串口初始化 */
	Debug_UART9_Init();
	
	adc_Init();

//    /* 电机初始化 */
    bldcm_init();

    printf("这是一个无刷电机基础控制示例\r\n");
    printf("打开串口助手发送以下指令，可控制电机运行状态：\r\n");
    printf("s----------------电机开始旋转\r\n");
    printf("p----------------电机停止旋转\r\n");
    printf("u----------------电机加速旋转[PWM+10%%]\r\n");
    printf("d----------------电机减速旋转[PWM-10%%]\r\n");
    printf("r----------------电机反向旋转\r\n");

    R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_SECONDS); //延时1秒

    Calculate_Current_Offset();


	
	while(1)
	{
	    if(uart_recv_motor_enable == true)
	    {
	        uart_recv_motor_enable = false;

	        /* 使能电机 */
	        ChannelPulse = 10;
	        set_bldcm_speed(ChannelPulse);
	        set_bldcm_enable();
	        GPT_Timing_Init();      //启动定时器
	    }
        if(uart_recv_motor_disenable == true)
        {
            uart_recv_motor_disenable = false;

            /* 停止电机 */
            set_bldcm_disable();
        }
        if(uart_recv_motor_speed_up == true)
        {
            uart_recv_motor_speed_up = false;

            static int is_run_flag;

            if(ChannelPulse==0)//占空比从零增加后 重新使能一次
            {
                is_run_flag=1;
            }

            /* 增大占空比 */
            ChannelPulse +=10;

            if(ChannelPulse >= 90)
                ChannelPulse = 100;

            set_bldcm_speed(ChannelPulse);

            if(is_run_flag==1)
            {
                set_bldcm_enable();
                is_run_flag=0;
            }
        }
        if(uart_recv_motor_speed_down == true)
        {
            uart_recv_motor_speed_down = false;

            /* 减小占空比 */
            if(ChannelPulse <= 10)
                ChannelPulse = 0;
            else
                ChannelPulse -= 10;

            set_bldcm_speed(ChannelPulse);
        }
        if(uart_recv_motor_reverse == true)
        {
            uart_recv_motor_reverse = false;

            Motor_Control_Reverse();
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
void R_BSP_WarmStart (bsp_warm_start_event_t event)
{
    if(BSP_WARM_START_RESET == event)
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
