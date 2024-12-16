#include "bsp_debug_uart.h"


// 定义 Order 变量，初始化为空字符
char Order = '\0';



/* 调试串口 UART9 初始化 */
void Debug_UART9_Init(void)
{
    fsp_err_t err = FSP_SUCCESS;

    err = R_SCI_B_UART_Open (&debug_uart9_ctrl, &debug_uart9_cfg);

    assert(FSP_SUCCESS == err);
}

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
            // 将接收到的的值转换为 char 类型，并赋值给 Order
            Order = (char)(p_args->data);
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



/* 函数声明 防止编译器警告 */
 int _isatty(int fd);
 int _close(int fd);
 int _lseek(int fd, int ptr, int dir);
 int _fstat(int fd, struct stat *st);

 __attribute__((weak)) int _isatty(int fd)
 {
     if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
         return 1;

     errno = EBADF;
     return 0;
 }

 __attribute__((weak)) int _close(int fd)
 {
     if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
         return 0;

     errno = EBADF;
     return -1;
 }

 __attribute__((weak)) int _lseek(int fd, int ptr, int dir)
 {
     (void) fd;
     (void) ptr;
     (void) dir;

     errno = EBADF;
     return -1;
 }

 __attribute__((weak)) int _fstat(int fd, struct stat *st)
 {
     if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
     {
         st->st_mode = S_IFCHR;
         return 0;
     }

     errno = EBADF;
     return 0;
 }
