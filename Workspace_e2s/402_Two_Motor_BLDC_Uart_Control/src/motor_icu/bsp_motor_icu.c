#include "motor_icu/bsp_motor_icu.h"

#include <stdio.h>
#include "led/bsp_led.h"
#include "motor_gpt/bsp_motor_gpt.h"
#include "motor_control/bsp_motor_control.h"


uint8_t Get_Hall_State(void);
uint8_t Motor2_Get_Hall_State(void);
bsp_io_level_t Get_Io_State(bsp_io_port_pin_t pin);

static uint16_t motor1_bldcm_pulse = 0;
static uint16_t motor2_bldcm_pulse = 0;

/**
  * @brief  霍尔引脚初始化
  * @param  无
  * @retval 无
  */
void motor1_hall_gpio_init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /*开启外部中断*/
    err = R_ICU_ExternalIrqOpen(&Motor1_EA_HU_ctrl, &Motor1_EA_HU_cfg);
    assert(FSP_SUCCESS == err);
    err = R_ICU_ExternalIrqOpen(&Motor1_EB_HV_ctrl, &Motor1_EB_HV_cfg);
    assert(FSP_SUCCESS == err);
    err = R_ICU_ExternalIrqOpen(&Motor1_EZ_HW_ctrl, &Motor1_EZ_HW_cfg);
    assert(FSP_SUCCESS == err);

}

/**
  * @brief  霍尔引脚初始化
  * @param  无
  * @retval 无
  */
void motor2_hall_gpio_init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /*开启外部中断*/
    err = R_ICU_ExternalIrqOpen(&Motor2_EA_HU_ctrl, &Motor2_EA_HU_cfg);
    assert(FSP_SUCCESS == err);
    err = R_ICU_ExternalIrqOpen(&Motor2_EB_HV_ctrl, &Motor2_EB_HV_cfg);
    assert(FSP_SUCCESS == err);
    err = R_ICU_ExternalIrqOpen(&Motor2_EZ_HW_ctrl, &Motor2_EZ_HW_cfg);
    assert(FSP_SUCCESS == err);

}

/**
  * @brief  霍尔引脚使能
  * @param  无
  * @retval 无
  */
void motor1_hall_enable(void)
{
    fsp_err_t err = FSP_SUCCESS;
    external_irq_callback_args_t casual;
    /*使能外部中断*/
    err = R_ICU_ExternalIrqEnable(&Motor1_EA_HU_ctrl);
    assert(FSP_SUCCESS == err);
    err = R_ICU_ExternalIrqEnable(&Motor1_EB_HV_ctrl);
    assert(FSP_SUCCESS == err);
    err = R_ICU_ExternalIrqEnable(&Motor1_EZ_HW_ctrl);
    assert(FSP_SUCCESS == err);

    motor_hal_callback(&casual);    // 执行一次换相
}

/**
  * @brief  霍尔引脚使能
  * @param  无
  * @retval 无
  */
void motor2_hall_enable(void)
{
    fsp_err_t err = FSP_SUCCESS;
    external_irq_callback_args_t casual;

    /*使能外部中断*/
    err = R_ICU_ExternalIrqEnable(&Motor2_EA_HU_ctrl);
    assert(FSP_SUCCESS == err);
    err = R_ICU_ExternalIrqEnable(&Motor2_EB_HV_ctrl);
    assert(FSP_SUCCESS == err);
    err = R_ICU_ExternalIrqEnable(&Motor2_EZ_HW_ctrl);
    assert(FSP_SUCCESS == err);

    motor2_hal_callback(&casual);   // 执行一次换相
}

/**
  * @brief  霍尔引脚失能
  * @param  无
  * @retval 无
  */
void motor1_hall_disable(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /*使能外部中断*/
    err = R_ICU_ExternalIrqDisable(&Motor1_EA_HU_ctrl);
    assert(FSP_SUCCESS == err);
    err = R_ICU_ExternalIrqDisable(&Motor1_EB_HV_ctrl);
    assert(FSP_SUCCESS == err);
    err = R_ICU_ExternalIrqDisable(&Motor1_EZ_HW_ctrl);
    assert(FSP_SUCCESS == err);

    err = R_ICU_ExternalIrqDisable(&Motor2_EA_HU_ctrl);
    assert(FSP_SUCCESS == err);
    err = R_ICU_ExternalIrqDisable(&Motor2_EB_HV_ctrl);
    assert(FSP_SUCCESS == err);
    err = R_ICU_ExternalIrqDisable(&Motor2_EZ_HW_ctrl);
    assert(FSP_SUCCESS == err);
}

/**
  * @brief  霍尔引脚失能
  * @param  无
  * @retval 无
  */
void motor2_hall_disable(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /*使能外部中断*/
    err = R_ICU_ExternalIrqDisable(&Motor2_EA_HU_ctrl);
    assert(FSP_SUCCESS == err);
    err = R_ICU_ExternalIrqDisable(&Motor2_EB_HV_ctrl);
    assert(FSP_SUCCESS == err);
    err = R_ICU_ExternalIrqDisable(&Motor2_EZ_HW_ctrl);
    assert(FSP_SUCCESS == err);
}


/**
  * @brief  设置电机PWM占空比
  * @param  占空比
  * @retval 无
  */
void motor1_set_pwm_pulse(uint16_t pulse)
{
  /* 设置定时器通道输出 PWM 的占空比 */
    motor1_bldcm_pulse = pulse;
}

/**
  * @brief  设置电机PWM占空比
  * @param  占空比
  * @retval 无
  */
void motor2_set_pwm_pulse(uint16_t pulse)
{
  /* 设置定时器通道输出 PWM 的占空比 */
    motor2_bldcm_pulse = pulse;
}

/**
  * @brief  获取IO状态
  * @param  引脚
  * @retval 状态
  */
bsp_io_level_t Get_Io_State(bsp_io_port_pin_t pin)
{
    bsp_io_level_t state;

    R_IOPORT_PinRead(&g_ioport_ctrl,pin,&state);

    return state;
}


/**
  * @brief  获取霍尔传感器状态
  * @param  无
  * @retval 状态
  */
uint8_t Get_Hall_State(void)
{

    uint8_t state = 0;
    /* 读取霍尔传感器 U 的状态 */
    if(Get_Io_State(BSP_IO_PORT_11_PIN_06) == BSP_IO_LEVEL_HIGH )
    {
        state |= 0x01U << 0;
    }

    /* 读取霍尔传感器 V 的状态 */
    if(Get_Io_State(BSP_IO_PORT_11_PIN_07) == BSP_IO_LEVEL_HIGH )
    {
        state |= 0x01U << 1;
    }

    /* 读取霍尔传感器 W 的状态 */
    if(Get_Io_State(BSP_IO_PORT_10_PIN_10) == BSP_IO_LEVEL_HIGH)
    {
        state |= 0x01U << 2;
    }

    return state;

}

/**
  * @brief  获取霍尔传感器状态
  * @param  无
  * @retval 状态
  */
uint8_t Motor2_Get_Hall_State(void)
{

    uint8_t state = 0;
    /* 读取霍尔传感器 U 的状态 */
    if(Get_Io_State(BSP_IO_PORT_11_PIN_08) == BSP_IO_LEVEL_HIGH )
    {
        state |= 0x01U << 0;
    }

    /* 读取霍尔传感器 V 的状态 */
    if(Get_Io_State(BSP_IO_PORT_11_PIN_09) == BSP_IO_LEVEL_HIGH )
    {
        state |= 0x01U << 1;
    }

    /* 读取霍尔传感器 W 的状态 */
    if(Get_Io_State(BSP_IO_PORT_12_PIN_14) == BSP_IO_LEVEL_HIGH)
    {
        state |= 0x01U << 2;
    }

    return state;

}

int motor1_update = 0;     // 定时器更新计数


/**
  * @brief  霍尔引脚中断-执行六步换向
  * @param  无
  * @retval 无
  */
void motor_hal_callback(external_irq_callback_args_t *p_args)
{
    uint8_t step = 0;
    step = Get_Hall_State();

    if(motor1_get_bldcm_direction() == MOTOR_FWD)
    {
        switch(step)
        {
            case 1:    /* U+ W- */
                GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_u_ctrl, motor1_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_HIGH);
                break;

            case 2:     /* V+ U- */
                GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_v_ctrl, motor1_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_HIGH);

                break;

            case 3:    /* V+ W- */
                GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_v_ctrl, motor1_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_HIGH);
                break;

            case 4:     /* W+ V- */
                GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_w_ctrl, motor1_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_HIGH);
                break;

            case 5:     /* U+  V- */
                GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_u_ctrl, motor1_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_HIGH);
                break;

            case 6:     /* W+ U- */
                GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_w_ctrl, motor1_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_HIGH);
                break;
        }
    }
    else
    {
        switch(step)
        {
            case 1:    /* W+ U- */
                GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_w_ctrl, motor1_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_HIGH);
                break;

            case 2:     /* U+ V- */
                GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_u_ctrl, motor1_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_HIGH);

                break;

            case 3:    /* W+ V- */
                GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_w_ctrl, motor1_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_HIGH);
                break;

            case 4:     /* V+ W- */
                GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_v_ctrl, motor1_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_HIGH);
                break;

            case 5:     /* V+ U- */
                GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_v_ctrl, motor1_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_HIGH);
                break;

            case 6:     /* U+ W- */
                GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor_u_ctrl, motor1_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_HIGH);
                break;
        }
    }
    motor1_update = 0;
}


int motor2_update = 0;     // 定时器更新计数
/**
  * @brief  霍尔引脚中断-执行六步换向
  * @param  无
  * @retval 无
  */
void motor2_hal_callback(external_irq_callback_args_t *p_args)
{
    uint8_t step = 0;
    step = Motor2_Get_Hall_State();

    if(motor2_get_bldcm_direction() == MOTOR_FWD)
    {
        switch(step)
        {
            case 1:    /* U+ W- */
                GPT_PWM_SetDuty(&motor2_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_04, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_05, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_u_ctrl, motor2_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06, BSP_IO_LEVEL_HIGH);
                break;

            case 2:     /* V+ U- */
                GPT_PWM_SetDuty(&motor2_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_05, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_v_ctrl, motor2_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_04, BSP_IO_LEVEL_HIGH);

                break;

            case 3:    /* V+ W- */
                GPT_PWM_SetDuty(&motor2_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_04, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_05, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_v_ctrl, motor2_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06, BSP_IO_LEVEL_HIGH);
                break;

            case 4:     /* W+ V- */
                GPT_PWM_SetDuty(&motor2_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_04, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_w_ctrl, motor2_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_05, BSP_IO_LEVEL_HIGH);
                break;

            case 5:     /* U+  V- */
                GPT_PWM_SetDuty(&motor2_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_04, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_u_ctrl, motor2_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_05, BSP_IO_LEVEL_HIGH);
                break;

            case 6:     /* W+ U- */
                GPT_PWM_SetDuty(&motor2_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_05, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_w_ctrl, motor2_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_04, BSP_IO_LEVEL_HIGH);
                break;
        }
    }
    else
    {
        switch(step)
        {
            case 1:    /* W+ U- */
                GPT_PWM_SetDuty(&motor2_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_05, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_w_ctrl, motor2_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_04, BSP_IO_LEVEL_HIGH);
                break;

            case 2:     /* U+ V- */
                GPT_PWM_SetDuty(&motor2_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_04, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_u_ctrl, motor2_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_05, BSP_IO_LEVEL_HIGH);

                break;

            case 3:    /* W+ V- */
                GPT_PWM_SetDuty(&motor2_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_04, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_w_ctrl, motor2_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_05, BSP_IO_LEVEL_HIGH);
                break;

            case 4:     /* V+ W- */
                GPT_PWM_SetDuty(&motor2_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_04, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_05, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_v_ctrl, motor2_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06, BSP_IO_LEVEL_HIGH);
                break;

            case 5:     /* V+ U- */
                GPT_PWM_SetDuty(&motor2_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_05, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_v_ctrl, motor2_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_04, BSP_IO_LEVEL_HIGH);
                break;

            case 6:     /* U+ W- */
                GPT_PWM_SetDuty(&motor2_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_04, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_05, BSP_IO_LEVEL_LOW);

                GPT_PWM_SetDuty(&motor2_u_ctrl, motor2_bldcm_pulse, GPT_IO_PIN_GTIOCA);
                R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_06, BSP_IO_LEVEL_HIGH);
                break;
        }
    }
    motor2_update = 0;
}


/**
  * @brief  定时器更新中断回调函数
  * @param  htim:定时器句柄
  * @retval 无
  */
void g_timer0_callback(timer_callback_args_t *p_args)
{
    if(motor1_get_bldcm_speed() > 0)    //在电机运行时 去判断
    {
        if (motor1_update++ > 1)        // 有一次在产生更新中断前霍尔传感器没有捕获到值
        {
            printf("\r\n堵转超时\r\n");
            motor1_update = 0;

            LED1_ON;                    // 点亮LED1表示堵转超时停止

            /* 堵转超时停止 PWM 输出 */
            motor1_set_bldcm_disable();
        }
    }
    else
    {
        motor1_update = 0;
    }

    if(motor2_get_bldcm_speed() > 0)    //在电机运行时 去判断
    {
        if (motor2_update++ > 1)        // 有一次在产生更新中断前霍尔传感器没有捕获到值
        {
            printf("\r\n堵转超时\r\n");
            motor2_update = 0;

            LED1_ON;                    // 点亮LED1表示堵转超时停止

            /* 堵转超时停止 PWM 输出 */
            motor2_set_bldcm_disable();
        }
    }
    else
    {
        motor2_update = 0;
    }
}


