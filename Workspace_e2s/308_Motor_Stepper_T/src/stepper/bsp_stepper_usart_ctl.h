#ifndef __DEBUG_USART_H
#define __DEBUG_USART_H

//#include "stm32f4xx.h"
//#include <stdio.h>
//
////串口波特率
//#define DEBUG_USART_BAUDRATE                    115200
//
////串口接收缓冲数组大小
//#define UART_RX_BUFFER_SIZE 256
//extern unsigned char UART_RxBuffer[UART_RX_BUFFER_SIZE];
//
////引脚定义
///*******************************************************/
//#define DEBUG_USART                             USART1
//#define DEBUG_USART_CLK_ENABLE()                __USART1_CLK_ENABLE();
//
//#define RCC_PERIPHCLK_UARTx                     RCC_PERIPHCLK_USART1
//#define RCC_UARTxCLKSOURCE_SYSCLK               RCC_USART1CLKSOURCE_SYSCLK
//
//#define DEBUG_USART_RX_GPIO_PORT                GPIOB
//#define DEBUG_USART_RX_GPIO_CLK_ENABLE()        __GPIOB_CLK_ENABLE()
//#define DEBUG_USART_RX_PIN                      GPIO_PIN_6
//#define DEBUG_USART_RX_AF                       GPIO_AF7_USART1
//
//#define DEBUG_USART_TX_GPIO_PORT                GPIOB
//#define DEBUG_USART_TX_GPIO_CLK_ENABLE()        __GPIOB_CLK_ENABLE()
//#define DEBUG_USART_TX_PIN                      GPIO_PIN_7
//#define DEBUG_USART_TX_AF                       GPIO_AF7_USART1
//
//#define DEBUG_USART_IRQHandler                  USART1_IRQHandler
//#define DEBUG_USART_IRQ                             USART1_IRQn
///************************************************************/
//
//void Usart_SendString(uint8_t *str);
//void DEBUG_USART_Config(void);
////int fputc(int ch, FILE *f);
//extern UART_HandleTypeDef UartHandle;
//extern void uart_FlushRxBuffer(void);
#endif /* __USART1_H */
