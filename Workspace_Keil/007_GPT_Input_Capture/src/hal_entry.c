#include "hal_data.h"
#include "led/bsp_led.h"
#include "beep/bsp_beep.h"
#include "key/bsp_key_irq.h"
#include "debug_uart/bsp_debug_uart.h"

#include "gpt/bsp_gpt_timing.h"
#include "gpt/bsp_gpt_pwm_output.h"
#include "gpt/bsp_gpt_input_capture.h"


FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER


extern timer_info_t info;  //用于获取定时器参数信息
extern uint32_t pwm_period;            //PWM周期
extern uint32_t pwm_high_level_time;   //PWM高电平的时间
extern uint32_t pwm_freq;              //PWM频率
extern uint32_t pwm_duty;              //PWM占空比


void hal_entry(void)
{
    /* TODO: add your own code here */

    LED_Init();         // LED 初始化
    Debug_UART9_Init(); // SCI9 UART 调试串口初始化

    GPT_PWM_Init();           // GPT PWM输出初始化
    GPT_InputCapture_Init();  // GPT 输入捕获初始化

    printf("这是一个 GPT 的PWM输出 + 输入捕获功能实验\r\n");
    printf("使用杜邦线连接 PE10 或 PE13 与 PB06  引脚，然后打开串口助手查看串口的打印信息\r\n");


    while(1)
    {
        /* 计算PWM的频率 */
        pwm_freq = info.clock_frequency / pwm_period;

        /* 计算PWM的占空比 */
        pwm_duty = pwm_high_level_time * 100 / pwm_period;

        // 打印
        printf("高电平时间=%lu, 周期=%u, ", pwm_high_level_time, pwm_period);
        printf("频率 = %ldkHz, 占空比 = %d%%\r\n", pwm_freq / 1000 , pwm_duty);
        pwm_period = pwm_high_level_time = pwm_freq = 0; //打印完后旧数据清零

        // LED1 闪烁指示程序正在运行...
        LED1_TOGGLE;
        // 间隔1s
        R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_SECONDS);
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
