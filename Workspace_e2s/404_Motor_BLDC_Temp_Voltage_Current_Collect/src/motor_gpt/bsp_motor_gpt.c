#include "motor_gpt/bsp_motor_gpt.h"

#include <stdio.h>
#include "led/bsp_led.h"
#include "motor_control/bsp_motor_control.h"


static uint16_t bldcm_pulse = 0;


/* 霍尔引脚GPT初始化函数 */
void GPT1_Init(void)
{
    /* 初始化 GPT 模块 */
    R_GPT_Open(&g_timer1_ctrl, &g_timer1_cfg);
    R_GPT_Open(&g_timer3_ctrl, &g_timer3_cfg);
}

/**
  * @brief  电机PWM定时器初始化
  * @param  无
  * @retval 无
  */
void motor_gpt_init(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* 初始化PWM定时器 */
    err = R_GPT_Open(&motor_u_ctrl, &motor_u_cfg);
    assert(FSP_SUCCESS == err);

    err = R_GPT_Open(&motor_v_ctrl, &motor_v_cfg);
    assert(FSP_SUCCESS == err);

    err = R_GPT_Open(&motor_w_ctrl, &motor_w_cfg);
    assert(FSP_SUCCESS == err);

    /* 启动 PWM1 定时器 */
    err = R_GPT_Start(&motor_u_ctrl);
    assert(FSP_SUCCESS == err);

    err = R_GPT_Start(&motor_v_ctrl);
    assert(FSP_SUCCESS == err);

    err = R_GPT_Start(&motor_w_ctrl);
    assert(FSP_SUCCESS == err);

}

/**
  * @brief  霍尔引脚使能
  * @param  无
  * @retval 无
  */
void hall_enable(void)
{
    timer_callback_args_t p_args;

    p_args.event = TIMER_EVENT_CAPTURE_A;

    /* 使能输入捕获 */
    R_GPT_Enable(&g_timer1_ctrl);
    R_GPT_Enable(&g_timer3_ctrl);

    /* 启动 GPT 定时器 */
    R_GPT_Start(&g_timer1_ctrl);
    R_GPT_Start(&g_timer3_ctrl);

    time1_callback(&p_args);                 // 执行一次换相
}

/**
  * @brief  霍尔引脚失能
  * @param  无
  * @retval 无
  */
void hall_disable(void)
{
    R_GPT_Stop(&g_timer1_ctrl);

    R_GPT_Disable(&g_timer1_ctrl);

    R_GPT_Stop(&g_timer3_ctrl);

    R_GPT_Disable(&g_timer3_ctrl);
}


/**
  * @brief  停止电机PWM输出
  * @param  无
  * @retval 无
  */
void stop_pwm_output(void)
{
    GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
    GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
    GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);

    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);
}


/**
  * @brief  定时器占空比设置函数
  * @param  定时器、占空比、定时器引脚
  * @retval 无
  */
void GPT_PWM_SetDuty(timer_ctrl_t * const p_ctrl,uint16_t duty,uint32_t const pin)
{
    timer_info_t infox;
    uint32_t current_period_counts;
    uint32_t duty_cycle_counts;

    if (duty > 100)
        duty = 100; //限制占空比范围：0~100

    /* 获得GPT的信息 */
    R_GPT_InfoGet(p_ctrl, &infox);

    /* 获得计时器一个周期需要的计数次数 */
    current_period_counts = infox.period_counts;

    /* 根据占空比和一个周期的计数次数计算GTCCR寄存器的值 */
    duty_cycle_counts = (uint32_t)(((uint64_t) current_period_counts * duty) / 100);

    /* 最后调用FSP库函数设置占空比 */
    R_GPT_DutyCycleSet(p_ctrl, duty_cycle_counts, pin);
}

/**
  * @brief  设置电机PWM占空比
  * @param  占空比
  * @retval 无
  */
void set_pwm_pulse(uint16_t pulse)
{
  /* 设置定时器通道输出 PWM 的占空比 */
    bldcm_pulse = pulse;
}



int update = 0;             // 定时器更新计数
extern __IO uint16_t ChannelPulse;


/**
  * @brief  霍尔引脚中断-执行六步换向
  * @param  无
  * @retval 无
  */

void time3_callback(timer_callback_args_t *p_args)
{
    if(p_args->event != TIMER_EVENT_CYCLE_END)
    {
        p_args->event = TIMER_EVENT_CAPTURE_A;
        time1_callback(p_args);
    }
}



void time1_callback(timer_callback_args_t *p_args)
{
    uint8_t step = 0;
    step = Get_Hall_State();

    if(p_args->event == TIMER_EVENT_CYCLE_END)
    {
        if(ChannelPulse > 0)            //防止在占空比减到0时，还在进行堵转判断
        {
            if (update++ >= 5)           // 有多次次在产生更新中断前霍尔传感器没有捕获到值
            {
                printf("\r\n堵转超时\r\n");

                update = 0;

                LED1_ON;                // 点亮LED1表示堵转超时停止

                /* 堵转超时停止 PWM 输出 */
                hall_disable();         // 禁用霍尔传感器接口
                stop_pwm_output();      // 停止 PWM 输出

                while(1);
            }
        }
    }

    if((p_args->event == TIMER_EVENT_CAPTURE_A) || (p_args->event == TIMER_EVENT_CAPTURE_B))
    {
        update = 0;
        if(get_bldcm_direction() == MOTOR_FWD)
        {
            switch(step)
            {
                case 1:    /* U+ W- */
                    GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_u_ctrl, bldcm_pulse, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_HIGH);
                    break;

                case 2:     /* V+ U- */
                    GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_v_ctrl, bldcm_pulse, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_HIGH);

                    break;

                case 3:    /* V+ W- */
                    GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_v_ctrl, bldcm_pulse, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_HIGH);
                    break;

                case 4:     /* W+ V- */
                    GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_w_ctrl, bldcm_pulse, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_HIGH);
                    break;

                case 5:     /* U+  V- */
                    GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_u_ctrl, bldcm_pulse, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_HIGH);
                    break;

                case 6:     /* W+ U- */
                    GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_w_ctrl, bldcm_pulse, GPT_IO_PIN_GTIOCA);
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

                    GPT_PWM_SetDuty(&motor_w_ctrl, bldcm_pulse, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_HIGH);
                    break;

                case 2:     /* U+ V- */
                    GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_u_ctrl, bldcm_pulse, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_HIGH);

                    break;

                case 3:    /* W+ V- */
                    GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_w_ctrl, bldcm_pulse, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_HIGH);
                    break;

                case 4:     /* V+ W- */
                    GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_v_ctrl, bldcm_pulse, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_HIGH);
                    break;

                case 5:     /* V+ U- */
                    GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_v_ctrl, bldcm_pulse, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_HIGH);
                    break;

                case 6:     /* U+ W- */
                    GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_u_ctrl, bldcm_pulse, GPT_IO_PIN_GTIOCA);
                    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_HIGH);
                    break;
            }
        }
    }

}
