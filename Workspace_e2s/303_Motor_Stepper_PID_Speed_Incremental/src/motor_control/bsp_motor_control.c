#include "bsp_motor_control.h"
#include "pid/bsp_pid.h"
#include "encoder/bsp_encoder.h"
#include "protocol/protocol.h"

_Bool motor_dir = true;                   // 电机方向，0表示正向，1表示反向。
_Bool motor_state = false;
extern volatile _Bool flag;           // 方向标志，true 为正转，false 为反转
extern volatile uint32_t pulse_period;   // 脉冲数

/*电机初始化*/
void Motor_Control_Init(void)
{

    fsp_err_t err = FSP_SUCCESS;
    /* 初始化 GPT 模块 */
    err = R_GPT_Open(&step_pwm_ctrl, &step_pwm_cfg);
    assert(FSP_SUCCESS == err);
    
    /* 设置电机方向 */
    STEP_CCW;
}

/**
 * @brief  启动电机
 */
void Motor_Control_Start(void)
{
    motor_state = true;
    STEP_ENBLED; // 使能电机
    R_GPT_Start(&step_pwm_ctrl);// 使能定时器7的PWM输出

}

/**
 * @brief  停止电机
 */
void Motor_Control_Stop(void)
{
    motor_state = false;
    STEP_DISENBLED;              // 禁用电机
    R_GPT_Stop(&step_pwm_ctrl);  // 禁用定时器7的PWM输出
}

/**
 * @brief  反转电机方向
 */
void Motor_Control_Reverse(void)
{
    motor_dir = !motor_dir;      // 切换电机方向
    STEP_DIRECTION_TOGGLE;      // 切换电机方向控制引脚
}

/**
 * @brief  控制电机速度
 * @param[in] speed_hz 目标速度，单位：Hz
 * @retval 无
 */
void Motor_Control_SetSpeed(uint32_t speed_hz)
{
    // 使用 R_GPT_PeriodSet 来设置定时器周期，从而控制 PWM 输出频率
    R_GPT_PeriodSet(&step_pwm_ctrl, Hz_Set(speed_hz));
}


/**
  * @brief  步进电机位置式PID控制
  * @retval 无
  * @note   基本定时器中断内调用
  */
void motor_pid_control(float actual_speed)
{
  uint32_t speed = (uint32_t)actual_speed;   // 将实际位置转换为整数类型

  /* 经过pid计算后的期望值 */
  volatile float cont_val = 0;

  /* 当电机运动时才启动pid计算 */
  if(motor_state == true )
  {
    /* 单位时间内的编码器脉冲数作为实际值传入pid控制器 */
    cont_val = PID_realize(actual_speed);// 进行 PID 计算

    // 控制值下限处理
    if (cont_val < 0)
    {
        cont_val = 0;    // 下限为 0
    }
    // 控制值上限处理
    else if (cont_val > PWM_MAX_FREQUENCY)
    {
        cont_val = PWM_MAX_FREQUENCY;    // 上限为 PWM 最大周期
    }

    // 设置电机速度
    Motor_Control_SetSpeed((uint32_t)cont_val);

    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &speed, 1);  // 给通道 1 发送实际值

  }
}


/* GPT定时器中断回调函数 */
void gpt0_timing_callback(timer_callback_args_t *p_args)
{
    static uint32_t last_pulse_period = 0; // 上一次的脉冲周期
    static uint32_t new_period = 0; // 上一次的脉冲周期

    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        if (pulse_period > 0 && pulse_period != last_pulse_period)
        {
            // 当前周期捕获脉冲值
            new_period = (pulse_period - last_pulse_period);

            motor_pid_control((float)new_period);
       }
        last_pulse_period = pulse_period; // 更新目前脉冲周期
    }
}
