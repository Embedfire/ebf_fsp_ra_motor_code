#ifndef R_BSP_MOTOR_CONTROL_H
#define R_BSP_MOTOR_CONTROL_H


#include "hal_data.h"

/* 电机方向枚举 */
typedef enum
{
  MOTOR_FWD = 0,
  MOTOR_REV,
}motor_dir_t;

typedef struct
{
  motor_dir_t direction;    //电机方向
  uint16_t dutyfactor;      // PWM占空比
  uint8_t is_enable;        // 电机使能
  uint32_t lock_timeout;    // 电机堵转计时间
}bldcm_data_t;

/* 电机 SD or EN 使能脚 */
#define BLDCM_ENABLE_SD()   R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_11_PIN_15, BSP_IO_LEVEL_HIGH) // 高电平打开-高电平使能
#define BLDCM_DISABLE_SD()  R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_11_PIN_15, BSP_IO_LEVEL_LOW)  // 低电平关断-低电平禁用

#define BLDCM_ENABLE_SD1()   R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_12_PIN_15, BSP_IO_LEVEL_HIGH) // 高电平打开-高电平使能
#define BLDCM_DISABLE_SD1()  R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_12_PIN_15, BSP_IO_LEVEL_LOW)  // 低电平关断-低电平禁用


void bldcm_init(void);
void bldcm_enable(void);

void Motor1_Control_Reverse(void);
void Motor2_Control_Reverse(void);
void motor1_set_bldcm_enable(void);
void motor2_set_bldcm_enable(void);
void motor1_set_bldcm_disable(void);
void motor2_set_bldcm_disable(void);
uint16_t motor1_get_bldcm_speed(void);
uint16_t motor2_get_bldcm_speed(void);
void motor1_set_bldcm_speed(uint16_t v);
void motor2_set_bldcm_speed(uint16_t v);
motor_dir_t motor1_get_bldcm_direction(void);
motor_dir_t motor2_get_bldcm_direction(void);
void motor1_set_bldcm_direction(motor_dir_t dir);
void motor2_set_bldcm_direction(motor_dir_t dir);




#endif
