#include "r_bldc.h"
#include "adc/bsp_adc_vr.h"

/*电机方向标志位*/
_Bool Bldc_Direction = true;

/* 无刷电机PWM初始化函数 */
void initMotor(void)
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
@brief 获取引脚状态

@param[in] pin ：引脚

@return 按键状态，类型为bsp_io_level_t

@note 该函数用于读取指定引脚的按键状态
**/
bsp_io_level_t Get_Key_State(bsp_io_port_pin_t pin)
{
    bsp_io_level_t state;
    
    R_IOPORT_PinRead(&g_ioport_ctrl,pin,&state);

    return state;
}


/**

@brief 获取霍尔传感器状态

@return 霍尔传感器状态，类型为uint8_t

@note 该函数用于读取三个霍尔传感器的状态，并将其合并为一个8位状态值返回
**/
uint8_t Get_Hall_State(void)
{
	
    uint8_t state = 0;
    /* 读取霍尔传感器 U 的状态 */
    if(Get_Key_State(BSP_IO_PORT_11_PIN_06) == BSP_IO_LEVEL_HIGH )
    {
        state |= 0x01U << 0;
    }
  
    /* 读取霍尔传感器 V 的状态 */
    if(Get_Key_State(BSP_IO_PORT_11_PIN_07) == BSP_IO_LEVEL_HIGH )
    {
        state |= 0x01U << 1;
    }
  
    /* 读取霍尔传感器 W 的状态 */
    if(Get_Key_State(BSP_IO_PORT_10_PIN_10) == BSP_IO_LEVEL_HIGH)
    {
        state |= 0x01U << 2;
    }
	
	return state; 
	
}

/** 自定义函数：设置PWM占空比
    @param duty 占空比范围：0~100 %
*/
void GPT_PWM_SetDuty(timer_ctrl_t * const p_ctrl,uint8_t duty,uint32_t const pin)      //1-2ms  20ms
{
    timer_info_t info;
    uint32_t current_period_counts;
    uint32_t duty_cycle_counts;

    if (duty > 100)
        duty = 100; //限制占空比范围：0~100

    /* 获得GPT的信息 */
    R_GPT_InfoGet(p_ctrl, &info);

    /* 获得计时器一个周期需要的计数次数 */
    current_period_counts = info.period_counts;

    /* 根据占空比和一个周期的计数次数计算GTCCR寄存器的值 */
    duty_cycle_counts = (uint32_t)(((uint64_t) current_period_counts * duty) / 100);

    /* 最后调用FSP库函数设置占空比 */
    R_GPT_DutyCycleSet(p_ctrl, duty_cycle_counts, pin);
}


/**

@brief 改变电机状态

@note 根据霍尔传感器状态和方向控制电机的工作状态
**/
void Change(void)
{

    uint8_t step = 0;
	uint8_t speed;
	
	speed = (uint8_t)((float)Read_ADC_Voltage_Value() / 4096 * 100);
	
    step = Get_Hall_State();
	
	if(Bldc_Direction == true)
	{

			switch(step)
			{
				case 1:    /* U+ W- */
                    GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);   					   
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);
				
					GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);  
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

                    GPT_PWM_SetDuty(&motor_u_ctrl, speed, GPT_IO_PIN_GTIOCA);
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_HIGH);
					break;
				
				case 2:     /* V+ U- */
					GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);    
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);

					GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);
				
					GPT_PWM_SetDuty(&motor_v_ctrl, speed, GPT_IO_PIN_GTIOCA);  
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_HIGH);
				
					break;
				
				case 3:    /* V+ W- */
					GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA); 	
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW);	

					GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);  	
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);	
					
					GPT_PWM_SetDuty(&motor_v_ctrl, speed, GPT_IO_PIN_GTIOCA); 
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_HIGH); 
					break;
				
				case 4:     /* W+ V- */
					GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA); 	
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

					GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);  	
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW); 
		 
					GPT_PWM_SetDuty(&motor_w_ctrl, speed, GPT_IO_PIN_GTIOCA);
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_HIGH);
					break;
				
				case 5:     /* U+  V- */
					GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA); 	
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW); 
				
					GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);    
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW); 
				
					GPT_PWM_SetDuty(&motor_u_ctrl, speed, GPT_IO_PIN_GTIOCA); 
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_HIGH);
					break;
				
				case 6:     /* W+ U- */
					GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA);  
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);
				
					GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA); 
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW); 
				
					GPT_PWM_SetDuty(&motor_w_ctrl, speed, GPT_IO_PIN_GTIOCA); 
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

                    GPT_PWM_SetDuty(&motor_w_ctrl, speed, GPT_IO_PIN_GTIOCA);	
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_HIGH);
					break;
				
				case 2:     /* U+ V- */
					GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);      
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

					GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);
				
					GPT_PWM_SetDuty(&motor_u_ctrl, speed, GPT_IO_PIN_GTIOCA); 
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_HIGH);	
				
					break;
				
				case 3:    /* W+ V- */
					GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA); 	
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW);	

					GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);  	
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);	
					
					GPT_PWM_SetDuty(&motor_w_ctrl, 50, GPT_IO_PIN_GTIOCA);  
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_HIGH); 
					break;
				
				case 4:     /* V+ W- */
					GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA); 
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW);

					GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA); 
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW); 
		 
					GPT_PWM_SetDuty(&motor_v_ctrl, speed, GPT_IO_PIN_GTIOCA); 
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_HIGH);
					break;
				
				case 5:     /* V+ U- */
					GPT_PWM_SetDuty(&motor_u_ctrl, 0, GPT_IO_PIN_GTIOCA); 				
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW); 				
				
					GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA);    			
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_LOW); 			
				
					GPT_PWM_SetDuty(&motor_v_ctrl, speed, GPT_IO_PIN_GTIOCA); 
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_HIGH);
					break;
				
				case 6:     /* U+ W- */
					GPT_PWM_SetDuty(&motor_w_ctrl, 0, GPT_IO_PIN_GTIOCA); 
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_13, BSP_IO_LEVEL_LOW); 
				
					GPT_PWM_SetDuty(&motor_v_ctrl, 0, GPT_IO_PIN_GTIOCA);   
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_14, BSP_IO_LEVEL_LOW); 	
				
					GPT_PWM_SetDuty(&motor_u_ctrl, speed, GPT_IO_PIN_GTIOCA);   
					R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_14_PIN_15, BSP_IO_LEVEL_HIGH);
					break;
			}
	
	}
}
	