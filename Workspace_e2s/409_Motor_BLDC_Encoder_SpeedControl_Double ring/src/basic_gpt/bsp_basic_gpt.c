#include "basic_gpt/bsp_basic_gpt.h"

#include "./led/bsp_led.h"
#include "./pid/bsp_pid.h"
#include "./protocol/protocol.h"
#include "motor_gpt/bsp_motor_gpt.h"
#include "motor_control/bsp_motor_control.h"

extern motor_rotate_t motor_drive;
extern bldcm_data_t bldcm_data;

#define TARGET_SPEED_MAX    3000    // 目标速度的最大值 r/m


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


void bldcm_pid_control(void)
{  // 电机旋转的当前位置

  if (bldcm_data.is_enable)
  {
        int cont_val = 0;    // 当前控制值

        cont_val = location_pid_realize(&pid_location);    // 进行 PID 计算

        /* 目标速度上限处理 */
        if (cont_val > TARGET_SPEED_MAX)
        {
            cont_val = TARGET_SPEED_MAX;
        }
        else if (cont_val < -TARGET_SPEED_MAX)
        {
            cont_val = -TARGET_SPEED_MAX;
        }

        set_pid_target(&pid_speed, cont_val);    // 设定位置的目标值

        cont_val = speed_pid_realize(&pid_speed);    // 进行 PID 计算
        if (cont_val < 0)
        {
                cont_val = -cont_val;
                bldcm_data.direction = MOTOR_REV;
        }
        else
        {
                bldcm_data.direction = MOTOR_FWD;
        }

        cont_val = (cont_val > 990) ? 990 : cont_val;  // 上限处理

        set_bldcm_speed(cont_val);

    }

}


void time0_callback(timer_callback_args_t *p_args)
{
    bldcm_pid_control();

    int x = (int)motor_drive.location;
    int x1 = (int)motor_drive.speed;

    if(bldcm_data.is_enable == 1)
    {
        set_computer_value(SEND_FACT_CMD, CURVES_CH1, &x, 1);
        set_computer_value(SEND_FACT_CMD, CURVES_CH2, &x1, 1);
    }
}




