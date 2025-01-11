#include <motor_controls/steering_gear.h>
#include "key/bsp_key_irq.h"

uint8_t Angle_Num = 0;//角度变量


/* 定时器初始化函数 */
void Motor_Init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /* 初始化 GPT 模块 */
    err = R_GPT_Open(&gear_pwm_ctrl, &gear_pwm_cfg);
    assert(FSP_SUCCESS == err);

    /* 启动 GPT 定时器 */
    err = R_GPT_Start(&gear_pwm_ctrl);
    assert(FSP_SUCCESS == err);
}


/*按键1中断回调函数*/
void key1_irq_callback(external_irq_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
    //判断角度区间
    Angle_Num = ((180 - Angle_Num) >= 30) ? (Angle_Num + 30) : Angle_Num;
    //逆时针转30度
    Motor_SetAngle(Angle_Num);
    //等待完成
    R_BSP_SoftwareDelay(200, BSP_DELAY_UNITS_MILLISECONDS);
}


/*按键2中断回调函数*/
void key2_irq_callback(external_irq_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);

    //判断角度区间
    Angle_Num = ((Angle_Num - 0) >= 30) ? (Angle_Num - 30) : Angle_Num;
    //顺时针转30度
    Motor_SetAngle(Angle_Num);
    //等待完成
    R_BSP_SoftwareDelay(200, BSP_DELAY_UNITS_MILLISECONDS);

}

/**
 * @brief 函数功能描述
 * @param[in] 参数1描述
 * @param[out] 参数2描述
 * @return 返回值描述
 * @note 备注信息
**/


/**
 * @brief 设置齿轮角度
 * @param[in] angle:角度值，-90 ~ 90度
 */
void Motor_SetAngle(uint32_t angle)
{
    timer_info_t info;
    uint32_t current_period_counts;

    /* 获得GPT的信息 */
    R_GPT_InfoGet(&gear_pwm_ctrl, &info);

    /* 获得计时器一个周期需要的计数次数 */
    current_period_counts = info.period_counts;

    /*将脉宽角度转换为定时器计数器数值*/
    angle = (uint32_t)((0.5 + angle / 180.0 * (2.5 - 0.5)) / 20.0 * current_period_counts);

    /*将期望角度的数值输入以PWM方式实现*/
    R_GPT_DutyCycleSet(&gear_pwm_ctrl, current_period_counts - angle, GPT_IO_PIN_GTIOCA_AND_GTIOCB);

}
