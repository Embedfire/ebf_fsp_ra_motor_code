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


void bldcm_init(void);
int get_motor_speed(void);
void set_bldcm_enable(void);
uint8_t Get_Hall_State(void);
void set_bldcm_disable(void);
void set_bldcm_speed(uint16_t v);
void Motor_Control_Reverse(void);
motor_dir_t get_bldcm_direction(void);
void set_bldcm_direction(motor_dir_t dir);
bsp_io_level_t Get_Io_State(bsp_io_port_pin_t pin);



#endif
