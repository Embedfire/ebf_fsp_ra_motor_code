#ifndef MOTOR_DC_BRUSH_H
#define MOTOR_DC_BRUSH_H

#include "hal_data.h"

/*有刷电机使能控制*/
#define  BRUSH_ENBLED	 	 R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_11_PIN_15, BSP_IO_LEVEL_HIGH)
#define  BRUSH_DISENBLED	 R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_11_PIN_15, BSP_IO_LEVEL_LOW)


/*有刷电机方向控制*/
#define  BRUSH_CW	 	 	 R_GPT_Stop(&brush_pwm2_ctrl); \
							 R_GPT_Start(&brush_pwm1_ctrl);

#define  BRUSH_CCW	 		 R_GPT_Stop(&brush_pwm1_ctrl); \
							 R_GPT_Start(&brush_pwm2_ctrl);



typedef struct motor_dc_brush_s
{
	/* data */
	bool motor_en;
	bool direction;
	uint8_t speed;
	float now_speed;
}DC_MOTOR_S;

extern DC_MOTOR_S dc_motor_s;



void initMotor(void);
void Speed_Up(void);
void Speed_Dowm(void);
void Set_Brush_Speed(uint8_t speed);
void Set_Brush_Direction(bool direction_val);


#endif
