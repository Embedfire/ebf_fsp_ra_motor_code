#include "basic_gpt/bsp_basic_gpt.h"

#include "./led/bsp_led.h"
#include "./pid/bsp_pid.h"
#include "./protocol/protocol.h"
#include "motor_gpt/bsp_motor_gpt.h"
#include "motor_control/bsp_motor_control.h"


extern motor_rotate_t motor_drive;
extern bldcm_data_t bldcm_data;


void basic_gpt0_Init(void)
{
    /* 初始化 GPT0 模块 */
    R_GPT_Open(&g_timer0_ctrl, &g_timer0_cfg);

    /* 启动 GPT0 定时器 */
    R_GPT_Start(&g_timer0_ctrl);

}

/**
  * @brief  电机位置式 PID 控制实现(定时调用)
  * @param  无
  * @retval 无
  */
int cont_val = 0;    // 当前控制值

void bldcm_pid_control(void)
{

    if (bldcm_data.is_enable)
    {
        cont_val = PID_realize();

        if (cont_val < 0)
        {
                cont_val = -cont_val;
                bldcm_data.direction = MOTOR_REV;
        }
        else
        {
                bldcm_data.direction = MOTOR_FWD;
        }


        cont_val = (cont_val > MAX_SPEED) ? MAX_SPEED : cont_val;  // 上限处理

        set_bldcm_speed((uint16_t)cont_val);
    }
}


void time0_callback(timer_callback_args_t *p_args)
{
    bldcm_pid_control();

    int x = (int)motor_drive.speed;

    if(bldcm_data.is_enable == 1)
        set_computer_value(SEND_FACT_CMD, CURVES_CH1, &x, 1);
}




