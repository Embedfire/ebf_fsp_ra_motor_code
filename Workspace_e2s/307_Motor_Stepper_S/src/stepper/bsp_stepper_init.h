#ifndef __BSP_STEP_MOTOR_INIT_H
#define	__BSP_STEP_MOTOR_INIT_H

#include "hal_data.h"



//Motor 方向
#define MOTOR_DIR_PIN                  	BSP_IO_PORT_14_PIN_10

//Motor 使能
#define MOTOR_EN_PIN                  	BSP_IO_PORT_13_PIN_12




#define HIGH 1		//高电平
#define LOW  0		//低电平

#define ON  0			//开
#define OFF !0			//关

#define CLOCKWISE 			1//顺时针
#define ANTI_CLOCKWISE	0//逆时针

//输出比较模式周期设置为0xFFFF
#define TIM_PERIOD                   0xFFFF


#define MOTOR_EN(x)	if(x)   \
						{R_IOPORT_PinWrite(&g_ioport_ctrl, MOTOR_EN_PIN, BSP_IO_LEVEL_LOW);}\
					else	\
						{R_IOPORT_PinWrite(&g_ioport_ctrl, MOTOR_EN_PIN, BSP_IO_LEVEL_HIGH);}

#define MOTOR_DIR(x)if(x)	\
						{R_IOPORT_PinWrite(&g_ioport_ctrl, MOTOR_DIR_PIN, BSP_IO_LEVEL_LOW);}\
                    else	\
                        {R_IOPORT_PinWrite(&g_ioport_ctrl, MOTOR_DIR_PIN, BSP_IO_LEVEL_LOW);}
														
	


void stepper_Init(void);
void stepper_start_run(void);
void channel_sw(int temp);

#endif
