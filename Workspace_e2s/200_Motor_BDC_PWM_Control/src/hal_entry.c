#include <motor_control/bsp_motor_control.h>
#include "hal_data.h"
#include "led/bsp_led.h"
#include "beep/bsp_beep.h"
#include "key/bsp_key_irq.h"
#include "debug_uart/bsp_debug_uart.h"
#include "gpt/bsp_gpt_pwm_output.h"
FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

void Process_Motor_Command(void);  // 处理电机命令的函数声明

extern char Order;  // 外部变量 Order 的声明，表示命令字符

/*电机控制参数*/
extern int8_t motor_pwm_duty;
extern _Bool motor_dir;

/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void)
{
       /* TODO: add your own code here */

       LED_Init();         // LED 初始化
       Debug_UART9_Init(); // SCI4 UART 调试串口初始化

       MOTOR_PRINT("这是一个有刷电机基础控制示例\r\n");
       MOTOR_PRINT("打开串口助手发送以下指令，可控制电机运行状态：\r\n");
       MOTOR_PRINT("S----------------电机开始旋转\r\n");
       MOTOR_PRINT("P----------------电机停止旋转\r\n");
       MOTOR_PRINT("U----------------电机加速旋转[PWM+10%%]\r\n");
       MOTOR_PRINT("D----------------电机减速旋转[PWM-10%%]\r\n");
       MOTOR_PRINT("R----------------电机反向旋转\r\n");

       /* 电机PWM初始化 */
       Motor_GPT_PWM_Init();
       Motor_Control_Init();
       LED1_ON;
       LED4_ON;

       while(1)
          {
           //处理用户输入的命令，执行相应的电机控制操作
           Process_Motor_Command();

          }
#if BSP_TZ_SECURE_BUILD
    /* Enter non-secure code */
    R_BSP_NonSecureEnter();
#endif
}

/**
 * @brief 处理用户输入的命令，执行相应的电机控制操作
 *
 * @return 无
 */
void Process_Motor_Command(void)
{
    switch (Order) {
        case 'S':  // 启动电机
            Clear_Order;// 命令被处理后清空
            Motor_Control_Start();  // 调用电机启动函数
            MOTOR_PRINT("电机启动，当前PWM占空比 = %d%%\r\n", motor_pwm_duty);
            break;

        case 'P':  // 停止电机
            Clear_Order;// 命令被处理后清空
            Motor_Control_Stop();  // 调用电机停止函数
            MOTOR_PRINT("************电机关闭************\r\n");
            break;

        case 'U':  // 增加电机PWM占空比（加速）
            Clear_Order;// 命令被处理后清空
            motor_pwm_duty += 10;  // 每次增加10%
            if (motor_pwm_duty > 100) {
                motor_pwm_duty = 100;  // 限制最大占空比为100%
                MOTOR_PRINT("已到达最大占空比，当前电机PWM占空比 = %d%%\r\n", motor_pwm_duty);
            } else {
                Motor_Control_SetDirAndDuty(motor_dir, (uint8_t)motor_pwm_duty);  // 更新占空比
                MOTOR_PRINT("电机加速，新的电机PWM占空比 = %d%%\r\n", motor_pwm_duty);
            }
            break;

        case 'D':  // 减少电机PWM占空比（减速）
            Clear_Order;// 命令被处理后清空
            if (motor_pwm_duty < 10) {  // 避免减到负值
                motor_pwm_duty = 0;
                MOTOR_PRINT("已到达最小占空比，当前电机PWM占空比 = %d%%\r\n", motor_pwm_duty);
            } else {
                motor_pwm_duty -= 10;  // 每次减少10%
                Motor_Control_SetDirAndDuty(motor_dir, (uint8_t)motor_pwm_duty);  // 更新占空比
                MOTOR_PRINT("电机减速，新的电机PWM占空比 = %d%%\r\n", motor_pwm_duty);
            }
            break;

        case 'R':  // 反转电机旋转方向
            Clear_Order;// 命令被处理后清空
            Motor_Control_Reverse();  // 调用电机方向反转函数
            if (motor_dir == 0) {
                MOTOR_PRINT("电机方向翻转，当前旋转方向：正向\n");
            } else {
                MOTOR_PRINT("电机方向翻转，当前旋转方向：反向\n");
            }
            break;

        default:  // 无效命令

            break;
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
