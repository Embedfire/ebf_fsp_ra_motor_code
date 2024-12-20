#ifndef __BSP_KEY_H
#define __BSP_KEY_H
#include "hal_data.h"

/* 两个按键引脚定义 */
#define KEY2_PIN    BSP_IO_PORT_11_PIN_14
#define KEY3_PIN    BSP_IO_PORT_12_PIN_02


/* 定义宏 KEY_ON 表示按键按下
   定义宏 KEY_OFF 表示按键没有按下
*/
#define KEY_ON  1
#define KEY_OFF 0


/* Key初始化函数 */
void Key_Init(void);
uint32_t Key_Scan(bsp_io_port_pin_t key);

#endif
