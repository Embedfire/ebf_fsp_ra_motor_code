/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
* other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
* EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
* SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
* SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
* this software. By using this software, you agree to the additional terms and conditions found by accessing the
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2021 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/
/***********************************************************************************************************************
* File Name   : mtr_main.c
* Description : The main function and the processes of motor control application layer
***********************************************************************************************************************/
/**********************************************************************************************************************
* History : DD.MM.YYYY Version
*         : 09.12.2021 1.00
***********************************************************************************************************************/

/***********************************************************************************************************************
* Includes <System Includes> , "Project Includes"
***********************************************************************************************************************/
#include <stdint.h>
#include "mtr_main.h"
#include "hal_data.h"
#include "ics2_RA6T2.h"
#include "r_mtr_ics.h"

/***********************************************************************************************************************
* Global variables
***********************************************************************************************************************/
float       g_f4_speed_ref = 0;
uint8_t     g_u1_motor_status;            /* Motor status */
uint8_t     com_u1_sw_userif;             /* User interface switch */
uint8_t     g_u1_sw_userif;               /* User interface switch */
uint8_t     com_u1_mode_system;           /* System mode */
uint8_t     g_u1_mode_system;             /* System mode */
uint16_t    g_u2_max_speed_rpm;
uint8_t     g_u1_stop_req;
uint16_t    g_u2_chk_error;
uint16_t    g_u2_vr1_ad;
uint16_t    g_u2_conf_hw;
uint16_t    g_u2_conf_sw;
uint16_t    g_u2_conf_tool;
uint8_t     g_u1_reset_req;             /* Reset request flag */
uint8_t     g_u1_sw_cnt;                /* Counter to remove chattering */
motor_cfg_t g_user_motor_cfg;
motor_sensorless_extended_cfg_t g_user_motor_sensorless_extended_cfg;
motor_speed_cfg_t g_user_motor_speed_cfg;
motor_speed_extended_cfg_t g_user_motor_speed_extended_cfg;
motor_current_cfg_t g_user_motor_current_cfg;
motor_current_extended_cfg_t g_user_motor_current_extended_cfg;
motor_angle_cfg_t g_user_motor_angle_cfg;
motor_estimate_extended_cfg_t g_user_motor_estimate_extended_cfg;
motor_driver_cfg_t g_user_motor_driver_cfg;
motor_driver_extended_cfg_t g_user_motor_driver_extended_cfg;
motor_current_motor_parameter_t g_user_motor_current_motor_parameter;
motor_current_design_parameter_t g_user_motor_current_design_parameter;

/***********************************************************************************************************************
* Private functions
***********************************************************************************************************************/
static void     motor_fsp_init (void);
static void     mtr_board_led_control (uint8_t u1_motor_status);
static uint8_t  mtr_remove_sw_chattering (uint8_t u1_sw, uint8_t u1_on_off);
static void     board_ui (void);           /* Board user interface */
static void     ics_ui (void);             /* ICS (Analyzer) user interface */
static void     software_init (void);      /* Software initialize */
static uint16_t get_vr1 (void);
static uint8_t  get_sw1 (void);
static uint8_t  get_sw2 (void);
static void     led1_on (void);
static void     led2_on (void);
static void     led1_off (void);
static void     led2_off (void);

/***********************************************************************************************************************
* han'shu'm     : mtr_init
* Description   : 无刷电机初始化
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
void mtr_init(void)
{
    g_u2_conf_hw = 0x0008;                        /* 0000000000001000b */
    g_u2_conf_sw = 0x0000;                        /* 0000000000000000b */
    g_u2_conf_tool = 0x0300;                      /* 0000011000000000b */

    motor_fsp_init();
    software_init();                              /* Initialize private global variables */
	

//   ics2_init(ICS_SCI9_P605_P606, ICS_BRR, ICS_INT_MODE);

    /* Execute reset event */
    g_motor_sensorless0.p_api->reset(g_motor_sensorless0.p_ctrl);
} /* End of function mtr_init() */

/***********************************************************************************************************************
* Function Name : mtr_main
* Description   : 电机控制主函数
* Arguments     : 根据初始化值进入不同的控制模式
* Return Value  : None
***********************************************************************************************************************/
void mtr_main(void)
{
    if (g_u1_sw_userif != com_u1_sw_userif)
    {
        g_u1_sw_userif = com_u1_sw_userif;
        if (ICS_UI == g_u1_sw_userif)
        {
            g_u1_mode_system = g_u1_motor_status;
        }
    }

    if (BOARD_UI == g_u1_sw_userif)
    {
        board_ui();                           /* 按键控制模式 */
    }
    else if (ICS_UI == g_u1_sw_userif)
    {
        ics_ui();                             /* ICS控制模式 */
    }
    else
    {
        /* Do Nothing */
    }
} /* End of function mtr_main() */

/***********************************************************************************************************************
* Function Name : board_ui
* Description   : 使用按键控制模式
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
static void board_ui(void)
{
    uint8_t u1_temp_sw_signal;

    /*======================*/
    /*     Mode control     */
    /*======================*/

    /* 读取电机状态 */
    g_motor_sensorless0.p_api->statusGet(g_motor_sensorless0.p_ctrl, &g_u1_motor_status);
    switch (g_u1_motor_status)
    {
        case MOTOR_SENSORLESS_CTRL_STOP:
            u1_temp_sw_signal = get_sw1();

            /* 查看按键一 */
            if ((MTR_FLG_SET == mtr_remove_sw_chattering(u1_temp_sw_signal, SW1_ON)) && (MTR_FLG_SET != g_u1_stop_req))
            {
                g_motor_sensorless0.p_api->run(g_motor_sensorless0.p_ctrl);
            }
            break;

        case MOTOR_SENSORLESS_CTRL_RUN:
            u1_temp_sw_signal = get_sw1();

            /* 查看按键一 */
            if ((MTR_FLG_SET == mtr_remove_sw_chattering(u1_temp_sw_signal, SW1_OFF)) || (MTR_FLG_CLR != g_u1_stop_req))
            {
                g_motor_sensorless0.p_api->stop(g_motor_sensorless0.p_ctrl);
            }
            break;

        case MOTOR_SENSORLESS_CTRL_ERROR:

            u1_temp_sw_signal = get_sw2();
            if ((SW_OFF == g_u1_reset_req) && (MTR_FLG_SET == mtr_remove_sw_chattering(u1_temp_sw_signal, SW2_ON)))
            {
                g_u1_reset_req = SW_ON;
            }
            else if ((SW_ON == g_u1_reset_req) && (MTR_FLG_SET == mtr_remove_sw_chattering(u1_temp_sw_signal, SW2_OFF)))
            {
                g_u1_reset_req = SW_OFF;
                g_motor_sensorless0.p_api->reset(g_motor_sensorless0.p_ctrl);
            }
            else
            {
                ;
            }
            break;

        default: /* Do nothing */
            break;
    }

    /*=============================*/
    /*      电机速度设置    */
    /*=============================*/
    g_f4_speed_ref = ((float)g_u2_vr1_ad - ADJUST_OFFSET) * VR1_SCALING; /* 从电位器读取速度参考 */
    g_motor_sensorless0.p_api->speedSet(g_motor_sensorless0.p_ctrl, g_f4_speed_ref);

    if ((g_f4_speed_ref > (-STOP_RPM)) && (g_f4_speed_ref < STOP_RPM)) /* Check SW1 */
    {
        g_u1_stop_req = MTR_FLG_SET;
    }
    else
    {
        g_u1_stop_req = MTR_FLG_CLR;
    }

    /***** LED 控制 *****/
    mtr_board_led_control(g_u1_motor_status);
} /* End of function board_ui */

/***********************************************************************************************************************
* Function Name : ics_ui
* Description   : 使用ICS控制界面
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
static void ics_ui(void)
{
    uint8_t u1_temp;

    mtr_set_com_variables();

    /*============================*/
    /*        Execute event       */
    /*============================*/
    u1_temp = com_u1_mode_system;

    if (g_u1_mode_system != u1_temp)
    {
        if (u1_temp > MOTOR_SENSORLESS_CTRL_EVENT_RESET)
        {
            com_u1_mode_system = g_u1_mode_system;
        }
        else
        {
            g_u1_mode_system = u1_temp;
            switch (g_u1_mode_system)
            {
                case MOTOR_SENSORLESS_CTRL_EVENT_STOP:
                    g_motor_sensorless0.p_api->stop(g_motor_sensorless0.p_ctrl);
                break;

                case MOTOR_SENSORLESS_CTRL_EVENT_RUN:
                    g_motor_sensorless0.p_api->run(g_motor_sensorless0.p_ctrl);
                break;

                case MOTOR_SENSORLESS_CTRL_EVENT_RESET:
                    g_motor_sensorless0.p_api->reset(g_motor_sensorless0.p_ctrl);
                break;

                default:
                    /* Do nothing */
                break;
            }
        }
    }

    g_motor_sensorless0.p_api->statusGet(g_motor_sensorless0.p_ctrl, &g_u1_motor_status);

    if (MOTOR_SENSORLESS_CTRL_EVENT_RESET == g_u1_mode_system)
    {
        if (MOTOR_SENSORLESS_CTRL_STOP == g_u1_motor_status)
        {
            software_init();                            /* Initialize private global variables for reset event */
        }
        else if (MOTOR_SENSORLESS_CTRL_ERROR == g_u1_motor_status)
        {
            g_u1_mode_system   = MOTOR_SENSORLESS_CTRL_EVENT_ERROR;
            com_u1_mode_system = MOTOR_SENSORLESS_CTRL_EVENT_ERROR;
        }
        else
        {
            /* Do nothing */
        }
    }

    /***** LED control *****/
    mtr_board_led_control(g_u1_motor_status);
} /* End of function ics_ui */

/***********************************************************************************************************************
* Function Name : software_init
* Description   : 初始化私有全局变量
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
static void software_init(void)
{
    g_u1_motor_status            = MOTOR_SENSORLESS_CTRL_STOP;
    g_u2_max_speed_rpm           = MTR_MAX_SPEED_RPM;
    g_u1_sw_userif               = CONFIG_DEFAULT_UI;
    g_u1_mode_system             = MOTOR_SENSORLESS_CTRL_EVENT_STOP;
    g_u1_reset_req               = SW_OFF;

    /* ICS variables initialization */
    com_u1_sw_userif             = CONFIG_DEFAULT_UI;
    com_u1_mode_system           = MOTOR_SENSORLESS_CTRL_EVENT_STOP;
    mtr_ics_variables_init();
    mtr_set_com_variables();

    MOTOR1_ENABLE;
} /* End of function software_init */

/***********************************************************************************************************************
* Function Name : g_poe_overcurrent
* Description   : POEG2 中断函数
* Arguments     : p_args - 回调参数
* Return Value  : None
***********************************************************************************************************************/
void g_poe_overcurrent(poeg_callback_args_t *p_args)
{
    if (NULL != p_args)
    {
        R_POEG_Reset(g_poeg0.p_ctrl);
        g_motor_sensorless0.p_api->errorSet(g_motor_sensorless0.p_ctrl, MOTOR_ERROR_OVER_CURRENT_HW);
        g_u2_chk_error |= MOTOR_ERROR_OVER_CURRENT_HW;
    }
} /* End of function g_poe_overcurrent */

/***********************************************************************************************************************
* Function Name : motor_fsp_init
* Description   : 初始化电机FSP模块
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
static void motor_fsp_init(void)
{
    R_POEG_Open(g_poeg0.p_ctrl, g_poeg0.p_cfg);
    g_motor_sensorless0.p_api->open(g_motor_sensorless0.p_ctrl, g_motor_sensorless0.p_cfg);

    g_user_motor_cfg = *(g_motor_sensorless0_ctrl.p_cfg);
    g_user_motor_sensorless_extended_cfg = *(motor_sensorless_extended_cfg_t *)g_user_motor_cfg.p_extend;
    g_user_motor_cfg.p_extend = &g_user_motor_sensorless_extended_cfg;
    g_motor_sensorless0_ctrl.p_cfg = &g_user_motor_cfg;

    g_user_motor_speed_cfg = *(g_motor_speed0_ctrl.p_cfg);
    g_user_motor_speed_extended_cfg = *(motor_speed_extended_cfg_t *)g_user_motor_speed_cfg.p_extend;
    g_user_motor_speed_cfg.p_extend = &g_user_motor_speed_extended_cfg;
    
    g_user_motor_current_cfg = *(g_motor_current0_ctrl.p_cfg);
    g_user_motor_current_extended_cfg = *(motor_current_extended_cfg_t *)g_user_motor_current_cfg.p_extend;
    g_user_motor_current_motor_parameter = *(g_user_motor_current_extended_cfg.p_motor_parameter);
    g_user_motor_current_design_parameter = *(g_user_motor_current_extended_cfg.p_design_parameter);
    g_user_motor_current_extended_cfg.p_motor_parameter = &g_user_motor_current_motor_parameter;
    g_user_motor_current_extended_cfg.p_design_parameter = &g_user_motor_current_design_parameter;
    g_user_motor_current_cfg.p_extend = &g_user_motor_current_extended_cfg;
    
    g_user_motor_angle_cfg = *(g_motor_angle0_ctrl.p_cfg);
    g_user_motor_estimate_extended_cfg = *(motor_estimate_extended_cfg_t *)g_user_motor_angle_cfg.p_extend;
    g_user_motor_angle_cfg.p_extend = &g_user_motor_estimate_extended_cfg;

    g_user_motor_driver_cfg = *(g_motor_driver0_ctrl.p_cfg);
    g_user_motor_driver_extended_cfg = *(motor_driver_extended_cfg_t *)g_user_motor_driver_cfg.p_extend;
    g_user_motor_driver_cfg.p_extend = &g_user_motor_driver_extended_cfg;
} /* End of function motor_fsp_init */

/***********************************************************************************************************************
* Function Name : mtr_callback_event
* Description   : 电机周期控制函数
* Arguments     : p_args - Callback argument
* Return Value  : None
***********************************************************************************************************************/
void mtr_callback_event(motor_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case MOTOR_CALLBACK_EVENT_SPEED_FORWARD:
            break;

        case MOTOR_CALLBACK_EVENT_SPEED_BACKWARD:
        {
            g_u2_vr1_ad = get_vr1();
            break;
        }

        case MOTOR_CALLBACK_EVENT_CURRENT_FORWARD:
        {
            if (MOTOR_SENSORLESS_CTRL_ERROR != g_u1_motor_status)
            {
                g_motor_sensorless0.p_api->errorCheck(g_motor_sensorless0.p_ctrl, &g_u2_chk_error);
            }
            break;
        }

        case MOTOR_CALLBACK_EVENT_CURRENT_BACKWARD:
        {
            mtr_ics_interrupt_process();
            break;
        }

        default:
            break;
    }
} /* End of function mtr_callback_event */

/***********************************************************************************************************************
* Function Name : mtr_board_led_control
* Description   : 根据电机状态设置LED模式
* Arguments     : u1_motor_status - 电机控制状态
* Return Value  : None
***********************************************************************************************************************/
static void mtr_board_led_control(uint8_t u1_motor_status)
{
    /***** LED control *****/
    if (MOTOR_SENSORLESS_CTRL_STOP == u1_motor_status)         /* Check motor status */
    {
        led1_off();                                     /* LED1 off */
        led2_off();                                     /* LED2 off */
    }
    else if (MOTOR_SENSORLESS_CTRL_RUN == u1_motor_status)      /* Check motor status */
    {
        led1_on();                                      /* LED1 on */
        led2_off();                                     /* LED2 off */
    }
    else if (MOTOR_SENSORLESS_CTRL_ERROR == u1_motor_status)       /* Check motor status */
    {
        led1_off();                                     /* LED1 off */
        led2_on();                                      /* LED2 on */
    }
    else
    {
        led1_on();                                      /* LED1 on */
        led2_on();                                      /* LED2 on */
    }
} /* End of function mtr_board_led_control */

/***********************************************************************************************************************
* Function Name : mtr_remove_sw_chattering
* Description   : 读取按键状态
* Arguments     : u1_sw - 按键引脚状态
*                 u1_on_off - 检测状态
* Return Value  : u1_remove_chattering_flg - 检测结果
***********************************************************************************************************************/
static uint8_t mtr_remove_sw_chattering(uint8_t u1_sw, uint8_t u1_on_off)
{
    uint8_t u1_remove_chattering_flg;

    u1_remove_chattering_flg = 0;
    if (u1_on_off == u1_sw)
    {
        g_u1_sw_cnt++;
        if (CHATTERING_CNT < g_u1_sw_cnt)
        {
            u1_remove_chattering_flg = MTR_FLG_SET;
            g_u1_sw_cnt = 0;
        }
    }
    else
    {
        g_u1_sw_cnt = 0;
    }

    return (u1_remove_chattering_flg);
} /* End of function mtr_remove_sw_chattering */

/***********************************************************************************************************************
* Function Name : get_vr1
* Description   : 电位器AD读取
* Arguments     : None
* Return Value  : 电位器AD值
***********************************************************************************************************************/
static uint16_t get_vr1(void)
{
    uint16_t ad_data;

    g_adc0.p_api->read(g_adc0.p_ctrl, MTR_ADCH_VR1, &ad_data);

    return (ad_data);
} /* End of function get_vr1 */

/***********************************************************************************************************************
* Function Name : get_sw1
* Description   : 读取按键一引脚状态
* Arguments     : None
* Return Value  : 按键一引脚状态
***********************************************************************************************************************/
static uint8_t get_sw1(void)
{
    uint8_t tmp_port;

    R_IOPORT_PinRead(&g_ioport_ctrl, MTR_PORT_SW1, &tmp_port);

    return (tmp_port);


} /* End of function get_sw1 */

/***********************************************************************************************************************
* Function Name : get_sw2
* Description   : 读取按键二引脚状态
* Arguments     : None
* Return Value  : 按键二引脚状态
***********************************************************************************************************************/
static uint8_t get_sw2(void)
{
    uint8_t tmp_port;

    R_IOPORT_PinRead(&g_ioport_ctrl, MTR_PORT_SW2, &tmp_port);

    return (tmp_port);
} /* End of function get_sw2 */

/***********************************************************************************************************************
* Function Name : led1_on
* Description   : 点亮LED1
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
static void led1_on(void)
{
    R_IOPORT_PinWrite(&g_ioport_ctrl, MTR_PORT_LED1, MTR_LED_ON);
} /* End of function led1_on */

/***********************************************************************************************************************
* Function Name : led2_on
* Description   : 点亮LED2
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
static void led2_on(void)
{
    R_IOPORT_PinWrite(&g_ioport_ctrl, MTR_PORT_LED2, MTR_LED_ON);
} /* End of function led2_on */


/***********************************************************************************************************************
* Function Name : led1_off
* Description   : 关闭LED1
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
static void led1_off(void)
{
    R_IOPORT_PinWrite(&g_ioport_ctrl, MTR_PORT_LED1, MTR_LED_OFF);
} /* End of function led1_off */

/***********************************************************************************************************************
* Function Name : led2_off
* Description   : 关闭LED2
* Arguments     : None
* Return Value  : None
***********************************************************************************************************************/
static void led2_off(void)
{
    R_IOPORT_PinWrite(&g_ioport_ctrl, MTR_PORT_LED2, MTR_LED_OFF);
} /* End of function led2_off */