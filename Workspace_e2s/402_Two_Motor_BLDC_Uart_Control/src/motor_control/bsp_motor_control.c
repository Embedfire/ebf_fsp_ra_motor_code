#include "motor_control/bsp_motor_control.h"

#include "motor_gpt/bsp_motor_gpt.h"
#include "motor_icu/bsp_motor_icu.h"


/* 私有变量*/
static bldcm_data_t motor1_bldcm_data;
static bldcm_data_t motor2_bldcm_data;

/*局部函数*/
static void sd_io_init(void);

/**
  * @brief  电机初始化
  * @param  无
  * @retval 无
  */
void bldcm_init(void)
{
    motor1_gpt_init();          // 电机控制定时器，引脚初始化
    motor2_gpt_init();          // 电机控制定时器，引脚初始化

    motor1_hall_gpio_init();    // 霍尔传感器初始化
    motor2_hall_gpio_init();    // 霍尔传感器初始化

    sd_io_init();               // SD引脚初始化
    Gpt0_Init();                // 堵转超时监控定时器
}

/**
  * @brief  电机使能
  * @param  无
  * @retval 无
  */
void bldcm_enable(void)
{
    motor1_set_bldcm_speed(0);
    motor2_set_bldcm_speed(0);

    motor1_set_bldcm_enable();
    motor2_set_bldcm_enable();
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
void motor1_set_bldcm_speed(uint16_t v)
{
    motor1_bldcm_data.dutyfactor = v;
    motor1_set_pwm_pulse(v);     // 设置速度
}

/**
  * @brief  设置电机速度
  * @param  v: 速度（占空比）
  * @retval 无
  */
void motor2_set_bldcm_speed(uint16_t v)
{
    motor2_bldcm_data.dutyfactor = v;
    motor2_set_pwm_pulse(v);     // 设置速度
}

/**
  * @brief  设置电机方向
  * @param  电机方向枚举
  * @retval 无
  */
void motor1_set_bldcm_direction(motor_dir_t dir)
{
    motor1_bldcm_data.direction = dir;
}

/**
  * @brief  设置电机方向
  * @param  电机方向枚举
  * @retval 无
  */
void motor2_set_bldcm_direction(motor_dir_t dir)
{
    motor2_bldcm_data.direction = dir;
}
/**
  * @brief  获取电机当前速度
  * @param  无
  * @retval 电机占空比
  */
uint16_t motor1_get_bldcm_speed(void)
{
    return motor1_bldcm_data.dutyfactor;
}
/**
  * @brief  获取电机当前速度
  * @param  无
  * @retval 电机占空比
  */
uint16_t motor2_get_bldcm_speed(void)
{
    return motor2_bldcm_data.dutyfactor;
}

/**
  * @brief  获取电机当前方向
  * @param  无
  * @retval 电机方向
  */
motor_dir_t motor1_get_bldcm_direction(void)
{
    return motor1_bldcm_data.direction;
}
/**
  * @brief  获取电机当前方向
  * @param  无
  * @retval 电机方向
  */
motor_dir_t motor2_get_bldcm_direction(void)
{
    return motor2_bldcm_data.direction;
}

/**
  * @brief  使能电机
  * @param  无
  * @retval 无
  */
void motor1_set_bldcm_enable(void)
{
    BLDCM_ENABLE_SD();
    R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);   //延时1ms
    motor1_hall_enable();                                   //使能霍尔传感器
    Gpt0_Open();
}

/**
  * @brief  使能电机
  * @param  无
  * @retval 无
  */
void motor2_set_bldcm_enable(void)
{
    BLDCM_ENABLE_SD1();
    R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);   //延时1ms
    motor2_hall_enable();                                   //使能霍尔传感器
    Gpt0_Open();
}

/**
  * @brief  禁用电机
  * @param  无
  * @retval 无
  */
void motor1_set_bldcm_disable(void)
{
    /* 禁用霍尔传感器接口 */
    motor1_hall_disable();

    /* 停止 PWM 输出 */
    motor1_stop_pwm_output();

    /* 关闭 MOS 管 */
    BLDCM_DISABLE_SD();

    motor1_set_bldcm_speed(0);

    Gpt0_Close();
}

/**
  * @brief  禁用电机
  * @param  无
  * @retval 无
  */
void motor2_set_bldcm_disable(void)
{
    /* 禁用霍尔传感器接口 */
    motor2_hall_disable();

    /* 停止 PWM 输出 */
    motor2_stop_pwm_output();

    BLDCM_DISABLE_SD1();

    motor2_set_bldcm_speed(0);

    Gpt0_Close();
}
/**
  * @brief  反转电机方向
  * @param  无
  * @retval 无
  */
void Motor1_Control_Reverse(void)
{
    if (motor1_get_bldcm_direction() == MOTOR_FWD) // 顺时针方向
    {
        motor1_set_bldcm_direction(MOTOR_REV);
    }
    else
    {
        motor1_set_bldcm_direction(MOTOR_FWD);
    }

}

/**
  * @brief  反转电机方向
  * @param  无
  * @retval 无
  */
void Motor2_Control_Reverse(void)
{
    if (motor2_get_bldcm_direction() == MOTOR_FWD) // 顺时针方向
    {
        motor2_set_bldcm_direction(MOTOR_REV);
    }
    else
    {
        motor2_set_bldcm_direction(MOTOR_FWD);
    }
}









