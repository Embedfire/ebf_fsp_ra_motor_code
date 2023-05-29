#include "hal_data.h"
#include "led/bsp_led.h"
#include "beep/bsp_beep.h"
#include "key/bsp_key_irq.h"
#include "debug_uart/bsp_debug_uart.h"
#include "motor/motor_control.h"
#include "gpt/bsp_gpt_pwm_output.h"
FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

extern volatile bool motor_start_flag;
extern volatile bool motor_stop_flag;
extern volatile bool motor_speedup_flag;
extern volatile bool motor_slowdown_flag;
extern volatile bool motor_reverse_flag;
extern int8_t motor_pwm_duty;
extern uint8_t motor_dir;

/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void)
{
    /* TODO: add your own code here */
    LED_Init();         // LED 初始化
       Debug_UART9_Init(); // SCI4 UART 调试串口初始化

       printf("这是一个有刷电机运行Demo\r\n");
       printf("打开串口助手发送以下指令，可控制电机运行状态：\r\n");
       printf("s----------------电机开始旋转");
       printf("p----------------电机停止旋转");
       printf("u----------------电机加速旋转");
       printf("d----------------电机减速旋转");
       printf("r----------------电机反向旋转");
       /* 电机PWM和编码器初始化 */
       Motor_GPT_PWM_Init();


       Motor_Control_Init();
       LED1_ON;
       LED4_ON;
       while(1)
          {
              if(motor_start_flag == true)
              {
                  motor_start_flag = false;

                  Motor_Control_Start();  //启动电机
              }

              if(motor_stop_flag == true)
              {
                  motor_stop_flag = false;

                  Motor_Control_Stop();   //停止电机
              }

              if(motor_speedup_flag == true)
              {
                  motor_speedup_flag = false;

                  motor_pwm_duty += 10;
                  if (motor_pwm_duty > 100)
                      motor_pwm_duty = 100;

                  Motor_Control_SetDirAndDuty(motor_dir, (uint8_t)motor_pwm_duty);

                  printf("now motor_pwm_duty = %d\r\n", motor_pwm_duty);
              }

              if(motor_slowdown_flag == true)
              {
                  motor_slowdown_flag = false;

                  motor_pwm_duty -= 10;
                  if (motor_pwm_duty < 0)
                      motor_pwm_duty = 0;

                  Motor_Control_SetDirAndDuty(motor_dir, (uint8_t)motor_pwm_duty);

                  printf("now motor_pwm_duty = %d\r\n", motor_pwm_duty);
              }

              if(motor_reverse_flag == true)
              {
                  motor_reverse_flag = false;

                  Motor_Control_Reverse();    //反转电机
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
