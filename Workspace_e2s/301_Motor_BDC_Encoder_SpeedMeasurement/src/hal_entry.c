#include <motor_control/bsp_motor_control.h>
#include "hal_data.h"
#include "led/bsp_led.h"
#include "beep/bsp_beep.h"
#include "key/bsp_key_irq.h"
#include "debug_uart/bsp_debug_uart.h"
#include "gpt/bsp_gpt_pwm_output.h"
#include "encoder/bsp_encoder.h"


FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

/* 声明外部变量，用于电机控制标志 */
extern volatile bool motor_start_flag;    // 电机启动标志
extern volatile bool motor_stop_flag;     // 电机停止标志
extern volatile bool motor_speedup_flag;  // 电机加速标志
extern volatile bool motor_slowdown_flag; // 电机减速标志
extern volatile bool motor_reverse_flag;  // 电机反转标志

extern int8_t motor_pwm_duty;    // 电机PWM占空比
extern _Bool motor_dir;          // 电机方向标志，`true`表示正转，`false`表示反转
extern _Bool flag;               // 编码器检测电机方向标志

_Bool encoder_printf = false;    // 编码器打印标志，`true`表示启用打印，`false`表示禁用打印


extern timer_info_t info;       //用于获取定时器参数信息
extern uint32_t pulse_period;   // 两次捕获的差值，一个脉冲的周期数
extern uint32_t phase_a_frequency;     // A相的频率


void Motor_polling_control(void);


/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void)
{


        /* TODO: add your own code here */
        LED_Init();         // LED 初始化
        Debug_UART9_Init(); // SCI4 UART 调试串口初始化

        /* 初始化软件并设置控制结构中定义的链接。 */
        R_ELC_Open(&g_elc_ctrl, &g_elc_cfg);
        /* 全局启用ELC中的事件链接。 */
        R_ELC_Enable(&g_elc_ctrl);

        printf("这是一个有刷电机基础控制示例\r\n");
        printf("打开串口助手发送以下指令，可控制电机运行状态：\r\n");
        printf("s----------------电机开始旋转\r\n");
        printf("p----------------电机停止旋转\r\n");
        printf("u----------------电机加速旋转[PWM+10%%]\r\n");
        printf("d----------------电机减速旋转[PWM-10%%]\r\n");
        printf("r----------------电机反向旋转\r\n");
        /* 电机PWM和编码器初始化 */
        Motor_GPT_PWM_Init();

        Motor_Control_Init();

        initEncoder();     //初始化编码器

        LED1_ON;
        LED4_ON;

        while(1)
        {
               Motor_polling_control();         //电机轮询串口控制

               /*每200毫秒打印*/
               R_BSP_SoftwareDelay(200, BSP_DELAY_UNITS_MILLISECONDS);

               if(encoder_printf == true)
               {
               /*打印电机方向、A相频率、电机转轴处速度、电机输出轴处速度*/
                   if(flag == true)
                       printf("当前电机正转：频率 = %ldHz,转轴速度 = %.2f转/秒,输出速度 = %.2f转/秒\n",phase_a_frequency,SHAFT_SPEED(phase_a_frequency),OUTPUT_SPEED(phase_a_frequency));
                   else
                       printf("当前电机反转：频率 = %ldHz,转轴速度 = %.2f转/秒,输出速度 = %.2f转/秒\n",phase_a_frequency,SHAFT_SPEED(phase_a_frequency),OUTPUT_SPEED(phase_a_frequency));

               /*频率、速度清零*/
               pulse_period = phase_a_frequency = 0;
               }
         }


#if BSP_TZ_SECURE_BUILD
    /* Enter non-secure code */
    R_BSP_NonSecureEnter();
#endif
}

void Motor_polling_control(void){
if(motor_start_flag == true) // 如果电机启动标志为真
{
    motor_start_flag = false; // 将电机启动标志设为假

    encoder_printf = true;

    Motor_Control_Start();  // 启动电机

    /* 启动编码器 */
    R_GPT_Start(&encoder_ctrl);

    printf("电机启动，当前PWM占空比 = %d%%\r\n", motor_pwm_duty);
}

if(motor_stop_flag == true) // 如果电机停止标志为真
{
    motor_stop_flag = false; // 将电机停止标志设为假

    Motor_Control_Stop();   // 停止电机

    encoder_printf = false;

    /* 停止编码器 */
    R_GPT_Stop(&encoder_ctrl);

    printf("************电机关闭************\r\n");
}

if(motor_speedup_flag == true) // 如果电机加速标志为真
{
    motor_speedup_flag = false; // 将电机加速标志设为假

    motor_pwm_duty += 10; // 增加电机的PWM占空比
    if (motor_pwm_duty > 100) {// 如果PWM占空比超过100
        motor_pwm_duty = 100; // 将PWM占空比限制为100
    printf("已到达最大占空比，当前电机PWM占空比 = %d%%\r\n", motor_pwm_duty);
}else{
    Motor_Control_SetDirAndDuty(motor_dir, (uint8_t)motor_pwm_duty); // 设置电机方向和占空比

    printf("电机加速，新的电机PWM占空比 = %d%%\r\n", motor_pwm_duty); // 打印当前的PWM占空比
}
}

if(motor_slowdown_flag == true) // 如果电机减速标志为真
{
    motor_slowdown_flag = false; // 将电机减速标志设为假

    motor_pwm_duty -= 10; // 减少电机的PWM占空比
    if (motor_pwm_duty < 0){// 如果PWM占空比小于0
        motor_pwm_duty = 0; // 将PWM占空比限制为0
        printf("已到达最小占空比，当前电机PWM占空比 = %d%%\r\n", motor_pwm_duty);
    }else{
    Motor_Control_SetDirAndDuty(motor_dir, (uint8_t)motor_pwm_duty); // 设置电机方向和占空比

    printf("电机减速，新的电机PWM占空比 = %d%%\r\n", motor_pwm_duty); // 打印当前的PWM占空比
    }
}

if(motor_reverse_flag == true) // 如果电机反转标志为真
{


    motor_reverse_flag = false; // 将电机反转标志设为假

    Motor_Control_Reverse();    // 反转电机

    if (motor_dir == 0) {
        printf("电机方向翻转，当前旋转方向：反转\n");
        //flag == true ? printf("正转\n") : printf("反转\n") ;

    }
    else {
        printf("电机方向翻转，当前旋转方向：正转\n");
        //flag == true ? printf("正转\n") : printf("反转\n") ;
    }
}
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
