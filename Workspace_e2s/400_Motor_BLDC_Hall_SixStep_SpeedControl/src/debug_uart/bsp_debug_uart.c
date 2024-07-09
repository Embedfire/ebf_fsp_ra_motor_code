#include "bsp_debug_uart.h"


/* 调试串口 UART9 初始化 */
void Debug_UART9_Init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    
    err = R_SCI_B_UART_Open (&debug_uart9_ctrl, &debug_uart9_cfg);

    assert(FSP_SUCCESS == err);
}


/* 发送完成标志 */
volatile bool uart_send_complete_flag = false;

volatile bool uart_recv_motor_enable = false;
volatile bool uart_recv_motor_disenable = false;
volatile bool uart_recv_motor_speed_up = false;
volatile bool uart_recv_motor_speed_down = false;
volatile bool uart_recv_motor_reverse = false;


/* 串口中断回调 */
void debug_uart9_callback (uart_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
            if(p_args->data == 's')
                uart_recv_motor_enable = true;
            if(p_args->data == 'p')
                uart_recv_motor_disenable = true;
            if(p_args->data == 'u')
                uart_recv_motor_speed_up = true;
            if(p_args->data == 'd')
                uart_recv_motor_speed_down = true;
            if(p_args->data == 'r')
                uart_recv_motor_reverse = true;

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





