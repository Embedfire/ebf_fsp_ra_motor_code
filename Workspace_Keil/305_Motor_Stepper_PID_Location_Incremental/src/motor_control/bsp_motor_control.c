#include "bsp_motor_control.h"
#include "pid/bsp_pid.h"
#include "encoder/bsp_encoder.h"
#include "protocol/protocol.h"

_Bool motor_dir = true;                   // 电机方向，0表示正向，1表示反向。
_Bool motor_state = false;
extern volatile int32_t pulse_period;   // 脉冲数
fsp_err_t err = FSP_SUCCESS;
/*电机初始化*/
void Motor_Control_Init(void)
{


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
void motor_pid_control(float actual_location)
{
  int32_t location = (int32_t)actual_location;   // 将实际位置转换为整数类型

  /* 经过pid计算后的期望值 */
  volatile float cont_val = 0;

  /* 当电机运动时才启动pid计算 */
  if(motor_state == true )
  {
    /* 将实际位置作为输入传入PID控制器进行计算 */
    cont_val = PID_realize((float)actual_location); // 进行 PID 计算

    // 根据计算结果控制步进电机的旋转方向
    if (cont_val > 0)
    {
        STEP_CW;  // 顺时针旋转
    }
    else
    {
        STEP_CCW; // 逆时针旋转
        cont_val = -cont_val; // 取绝对值
    }

    // 控制值上限处理
    cont_val >= PWM_MAX_FREQUENCY ? (cont_val = PWM_MAX_FREQUENCY) : cont_val;

    // 设置步进电机速度
    Motor_Control_SetSpeed((uint32_t)cont_val);

    // 给通道 1 发送实际位置值
    set_computer_value(SEND_FACT_CMD, CURVES_CH1, &location, 1);
  }
}


/* GPT定时器中断回调函数 */
void gpt0_timing_callback(timer_callback_args_t *p_args)
{

    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        // 调用PID控制函数调整电机状态
        motor_pid_control((float)pulse_period);
    }
}
