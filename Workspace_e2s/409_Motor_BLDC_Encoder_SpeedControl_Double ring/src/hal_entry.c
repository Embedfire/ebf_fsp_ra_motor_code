#include "hal_data.h"

FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

#include "led/bsp_led.h"
#include "./pid/bsp_pid.h"
#include "./protocol/protocol.h"
#include "motor_gpt/bsp_motor_gpt.h"
#include "basic_gpt/bsp_basic_gpt.h"
#include "debug_uart/bsp_debug_uart.h"
#include "motor_control/bsp_motor_control.h"

void Key_IRQ_Init(void);

extern motor_rotate_t motor_drive;

/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void)
{
    int target_speed = 200;
    /* TODO: add your own code here */
    /* LED初始化 */
	LED_Init();

	Key_IRQ_Init();

	/* 串口初始化 */
	Debug_UART9_Init();

    /* 协议初始化 */
    protocol_init();

    PID_param_init();

    /* 电机初始化 */
    bldcm_init();
    basic_gpt0_Init();

    set_computer_value(SEND_STOP_CMD, CURVES_CH1, NULL, 0);                // 同步上位机的启动按钮状态
    set_computer_value(SEND_TARGET_CMD, CURVES_CH1, &target_speed, 1);     // 给通道 1 发送目标值


	while(1)
	{
	    /* 接收数据处理 */
	    receiving_process();

	}
	
	
#if BSP_TZ_SECURE_BUILD
    /* Enter non-secure code */
    R_BSP_NonSecureEnter();
#endif
}

/* KEY 外部中断初始化函数 */
void Key_IRQ_Init(void)
{
   fsp_err_t err = FSP_SUCCESS;

   /* Open ICU module */
   err = R_ICU_ExternalIrqOpen(&g_external_irq0_ctrl, &g_external_irq0_cfg);
   assert(FSP_SUCCESS == err);
   err = R_ICU_ExternalIrqOpen(&g_external_irq1_ctrl, &g_external_irq1_cfg);
   assert(FSP_SUCCESS == err);


   /* 允许中断 */
   err = R_ICU_ExternalIrqEnable(&g_external_irq0_ctrl);
   assert(FSP_SUCCESS == err);
   err = R_ICU_ExternalIrqEnable(&g_external_irq1_ctrl);
   assert(FSP_SUCCESS == err);
}

/* 按键中断回调函数 */
void key_callback(external_irq_callback_args_t *p_args)
{
   /* 判断中断通道 */
   if (4 == p_args->channel)
   {
//       uart_recv_motor_enable = true;   // 按键KEY1_SW2按下
   }
   else if (13 == p_args->channel)
   {
//       uart_recv_motor_disenable = true;   // 按键KEY2_SW3按下
   }
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
