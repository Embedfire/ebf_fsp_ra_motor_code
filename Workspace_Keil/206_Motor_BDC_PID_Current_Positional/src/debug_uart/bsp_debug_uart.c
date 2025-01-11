#include "bsp_debug_uart.h"

char Order;

/* 调试串口 UART9 初始化 */
void Debug_UART9_Init(void)
{
    fsp_err_t err = FSP_SUCCESS;

    err = R_SCI_B_UART_Open (&debug_uart9_ctrl, &debug_uart9_cfg);

    assert(FSP_SUCCESS == err);
}

uint8_t data[1];

/* 发送完成标志 */
volatile bool uart_send_complete_flag = false;
volatile bool uart_receive_complete_flag = false;

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


/**
 * @brief  向上位机发送数据包
 *
 * 通过UART将数据包发送到上位机，并等待发送完成。
 *
 * @param[in] p_src   指向待发送数据包的指针
 * @param[in] bytes   数据包的字节数
 */
void uart_protocol_send(uint8_t const * const p_src, uint32_t const bytes)
{
    // 调用UART写函数发送数据包
    R_SCI_B_UART_Write(&debug_uart9_ctrl, (uint8_t *)p_src, (uint32_t)bytes);

    // 等待数据包发送完成
    while (uart_send_complete_flag == false);

    // 重置发送完成标志
    uart_send_complete_flag = false;
}



#if defined __GNUC__ && !defined __clang__
int _write(int fd, char *pBuffer, int size); //防止编译警告
int _read(int fd, char *pBuffer, int size);

/* 重定向 printf 输出 */
int _write(int fd, char *pBuffer, int size)
{
    FSP_PARAMETER_NOT_USED ( fd );
   R_SCI_B_UART_Write (&debug_uart9_ctrl, (uint8_t*) pBuffer, (uint32_t) size);
   while (uart_send_complete_flag == false)
        ;
   uart_send_complete_flag = false;

   return size;
}

/* 重定向scanf函数 */
int _read(int fd, char *pBuffer, int size)
{
    FSP_PARAMETER_NOT_USED ( fd );

    R_SCI_B_UART_Read (&debug_uart9_ctrl, (uint8_t*) pBuffer, (uint32_t) size);
   while (uart_receive_complete_flag == false)
        ;
   uart_receive_complete_flag = false;

   /* 回显 */
   R_SCI_B_UART_Write (&debug_uart9_ctrl, (uint8_t*) pBuffer, (uint32_t) size);

   return size;
}

#else
int fputc(int ch, FILE *f)
{
    FSP_PARAMETER_NOT_USED ( f );
   R_SCI_B_UART_Write(&debug_uart9_ctrl, (uint8_t *)&ch, 1);
   while(uart_send_complete_flag == false);
   uart_send_complete_flag = false;

   return ch;
}
#endif
