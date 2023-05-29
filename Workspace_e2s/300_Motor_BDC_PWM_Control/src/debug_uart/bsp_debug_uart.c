#include "bsp_debug_uart.h"


/* 调试串口 UART9 初始化 */
void Debug_UART9_Init(void)
{
    fsp_err_t err = FSP_SUCCESS;

    err = R_SCI_B_UART_Open (&debug_uart9_ctrl, &debug_uart9_cfg);

    assert(FSP_SUCCESS == err);
}
// 电机功能操作允许标志位
volatile bool motor_start_flag = false;
volatile bool motor_stop_flag = false;
volatile bool motor_speedup_flag = false;
volatile bool motor_slowdown_flag = false;
volatile bool motor_reverse_flag = false;

/* 发送完成标志 */
volatile bool uart_send_complete_flag = false;


/* 串口中断回调 */
void debug_uart9_callback (uart_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
            /* 根据字符指令进行操作 */
            switch (p_args->data)
            {
                case 's':   //start
                    motor_start_flag = true;
                    break;
                case 'p':   //pause
                    motor_stop_flag = true;
                    break;
                case 'u':   //up
                    motor_speedup_flag = true;
                    break;
                case 'd':   //down
                    motor_slowdown_flag = true;
                    break;
                case 'r':   //reverse
                    motor_reverse_flag = true;
                    break;
                default:
                    // input error
                    break;
            }
            break;
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





