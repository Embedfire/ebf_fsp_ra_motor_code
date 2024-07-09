#include "motor_control/bsp_motor_control.h"

#include "motor_gpt/bsp_motor_gpt.h"
#include "motor_icu/bsp_motor_icu.h"


/* 私有变量*/
static bldcm_data_t bldcm_data;

/*局部函数*/
static void sd_io_init(void);


/**
  * @brief  电机初始化
  * @param  无
  * @retval 无
  */
void bldcm_init(void)
{
    motor_gpt_init();   // 电机控制定时器，引脚初始化
    hall_gpio_init();   // 霍尔传感器初始化
    sd_io_init();       // SD引脚初始化
    Gpt0_Init();        // 堵转超时监控定时器
}

/**
  * @brief  SD引脚初始化
  * @param  无
  * @retval 无
  */
static void sd_io_init(void)
{
    /* 初始化配置引脚（这里重复初始化了，可以注释掉） */
    R_IOPORT_Open (&g_ioport_ctrl, g_ioport.p_cfg);
}


/**
  * @brief  设置电机速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void set_bldcm_speed(uint16_t v)
{
    bldcm_data.dutyfactor = v;
    set_pwm_pulse(v);     // 设置速度
}

/**
  * @brief  设置电机方向
  * @param  电机方向枚举
  * @retval 无
  */
void set_bldcm_direction(motor_dir_t dir)
{
    bldcm_data.direction = dir;
}

/**
  * @brief  获取电机当前方向
  * @param  无
  * @retval 电机方向
  */
motor_dir_t get_bldcm_direction(void)
{
    return bldcm_data.direction;
}

/**
  * @brief  反转电机方向
  * @param  无
  * @retval 无
  */
void Motor_Control_Reverse(void)
{
    if (get_bldcm_direction() == MOTOR_FWD) // 顺时针方向
    {
        set_bldcm_direction(MOTOR_REV);
    }
    else
    {
        set_bldcm_direction(MOTOR_FWD);
    }

}


/**
  * @brief  使能电机
  * @param  无
  * @retval 无
  */
void set_bldcm_enable(void)
{
    BLDCM_ENABLE_SD();
    R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);   //延时1ms
    hall_enable();                                          //使能霍尔传感器
    Gpt0_Open();
}

/**
  * @brief  禁用电机
  * @param  无
  * @retval 无
  */
void set_bldcm_disable(void)
{
    /* 禁用霍尔传感器接口 */
    hall_disable();

    /* 停止 PWM 输出 */
    stop_pwm_output();

    /* 关闭 MOS 管 */
    BLDCM_DISABLE_SD();

    Gpt0_Close();
}







