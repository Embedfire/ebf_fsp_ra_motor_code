#include "bsp_debug_uart.h"

#include "motor_control/bsp_motor_control.h"

int speed;
uint8_t dec;
uint8_t motor1_one_flag = 1;
uint8_t motor2_one_flag = 1;
uint8_t uart_len = 0;
uint8_t uart_buffer[64] = {0};

volatile bool uart_recv_flag = false;

void Clear_buffer(void);

/* 调试串口 UART9 初始化 */
void Debug_UART9_Init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    
    err = R_SCI_B_UART_Open (&debug_uart9_ctrl, &debug_uart9_cfg);

    assert(FSP_SUCCESS == err);
}

void deal_uart_data(void)
{
    if((uart_buffer[0] == 'v') || (uart_buffer[0] == 'V'))      //速度
    {
        if(uart_buffer[1] == '1')
        {
            if (uart_buffer[2] == ' ')
            {
                speed = atoi((char const *)uart_buffer+2);
                if((speed >= 0) && (speed <= 100))
                {
                    motor1_set_bldcm_speed(speed);
                    printf("\n\r电机1速度: %d\n\r", speed);
                    Clear_buffer();

                    if(speed == 0)
                    {
                        motor1_one_flag = 1;
                    }
                    //第一次启动  需要主动换向
                    if((motor1_one_flag == 1) && (speed != 0))
                    {
                        motor1_one_flag = 0;
                        motor1_set_bldcm_enable();
                    }
                    return;
                }
                else
                {
                    printf("\n\r指令错误\n\r");
                    Clear_buffer();
                }
            }
        }
        else if(uart_buffer[1] == '2')
        {
            if (uart_buffer[2] == ' ')
            {
                speed = atoi((char const *)uart_buffer+2);
                if((speed >= 0) && (speed <= 100))
                {
                    motor2_set_bldcm_speed(speed);
                    printf("\n\r电机2速度: %d\n\r", speed);
                    Clear_buffer();

                    if(speed == 0)
                    {
                        motor2_one_flag = 1;
                    }
                    //第一次启动  需要主动换向
                    if((motor2_one_flag == 1) && (speed != 0))
                    {
                        motor2_one_flag = 0;
                        motor2_set_bldcm_enable();
                    }
                }
                else
                {
                    printf("\n\r指令错误\n\r");
                    Clear_buffer();
                }
            }
        }
    }
    else if((uart_buffer[0] == 'd') || (uart_buffer[0] == 'D')) //方向
    {
        if(uart_buffer[1] == '1')
        {
            if (uart_buffer[2] == ' ')
            {
                if((uart_buffer[3] == '0') || (uart_buffer[3] == '1'))
                {
                    if(uart_buffer[3] == '0')
                    {
                        motor1_set_bldcm_direction(MOTOR_FWD);
                        printf("\n\r电机1方向: 0\n\r");
                        Clear_buffer();
                    }
                    else
                    {
                        motor1_set_bldcm_direction(MOTOR_REV);
                        printf("\n\r电机1方向: 1\n\r");
                        Clear_buffer();
                    }
                }
                else
                {
                    printf("\n\r指令错误\n\r");
                    Clear_buffer();
                }
            }
        }
        else if(uart_buffer[1] == '2')
        {
            if (uart_buffer[2] == ' ')
            {
                if((uart_buffer[3] == '0') || (uart_buffer[3] == '1'))
                {
                    if(uart_buffer[3] == '0')
                    {
                        motor2_set_bldcm_direction(MOTOR_FWD);
                        printf("\n\r电机2方向: 0\n\r");
                        Clear_buffer();
                    }
                    else
                    {
                        motor2_set_bldcm_direction(MOTOR_REV);
                        printf("\n\r电机2方向: 1\n\r");
                        Clear_buffer();
                    }
                }
                else
                {
                    printf("\n\r指令错误\n\r");
                    Clear_buffer();
                }
            }
        }
    }
    else
    {
        printf("\n\r指令错误\n\r");
        Clear_buffer();
    }
}

void Clear_buffer(void)
{
    memset(uart_buffer,0,64);
    uart_len = 0;
}
/* 发送完成标志 */
volatile bool uart_send_complete_flag = false;


/* 串口中断回调 */
void debug_uart9_callback (uart_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
            uart_buffer[uart_len++] = p_args->data;

            //如果为回车键，则开始处理串口数据
            if(p_args->data == 10)
            {
                uart_recv_flag = true;
            }
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





