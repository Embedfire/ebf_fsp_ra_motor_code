#include "bsp_debug_uart.h"

#include "led/bsp_led.h"
#include "./protocol/protocol.h"

/* 调试串口 UART9 初始化 */
void Debug_UART9_Init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    
    err = R_SCI_B_UART_Open (&debug_uart9_ctrl, &debug_uart9_cfg);

    assert(FSP_SUCCESS == err);
}


/* 发送完成标志 */
volatile bool uart_send_complete_flag = false;


uint8_t data[1];

/* 串口中断回调 */
void debug_uart9_callback (uart_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {

            data[0] = (uint8_t)p_args->data;
            protocol_data_recv(data, 1);

            break;
        }
        case UART_EVENT_TX_COMPLETE:
        {
            uart_send_complete_flag = true;
            break;
        }
        default:
            break;
    }
}


void uart_protocol_send(uint8_t const * const p_src, uint32_t const bytes)
{
    R_SCI_B_UART_Write(&debug_uart9_ctrl, (uint8_t *)p_src, (uint32_t)bytes);

    while(uart_send_complete_flag == false);

    uart_send_complete_flag = false;
}



/* 重定向 printf 输出 */
#if defined __GNUC__ && !defined __clang__
int _write(int fd, char *pBuffer, int size); //防止编译警告
int _write(int fd, char *pBuffer, int size)
{
    (void)fd;
    R_SCI_B_UART_Write(&debug_uart9_ctrl, (uint8_t *)pBuffer, (uint32_t)size);
    while(uart_send_complete_flag == false);
    uart_send_complete_flag = false;

    return size;
}
#else
int fputc(int ch, FILE *f)
{
    (void)f;
    R_SCI_B_UART_Write(&debug_uart9_ctrl, (uint8_t *)&ch, 1);
    while(uart_send_complete_flag == false);
    uart_send_complete_flag = false;

    return ch;
}
#endif





