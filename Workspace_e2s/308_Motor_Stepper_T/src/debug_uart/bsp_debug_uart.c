#include "bsp_debug_uart.h"
#include "stepper/bsp_stepper_init.h"
#include "stepper/bsp_stepper_T_speed.h"
#include "led/bsp_led.h"

void Clear_buffer(void);
void MSD_demo_run(void);
void ShowHelp(void);
void ShowData(int position, int accel_val, int decel_val, int speed, int steps);

uint8_t uart_len = 0;
uint8_t uart_buffer[64] = {0};
volatile bool uart_recv_flag = false;


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

void Clear_buffer(void)
{
    memset(uart_buffer,0,64);
    uart_len = 0;
}
void deal_uart_data(void)
{
    static char showflag =1;
    //默认移动步数
    static int steps = SPR*5;
    //默认加速度
    static int accel_val = 500;
    //默认减速度
    static int decel_val = 100;
    //默认最大速度
    static int speed = 1000;

    int acc_temp=0;
    int dec_temp=0;
    int speed_temp=0;

    uint8_t okCmd = false;

    if((uart_buffer[0] == 'm'))      //速度
    {
        if(uart_buffer[1] == ' ')
        {
            steps = atoi((char const *)uart_buffer+2);
            stepper_move_T(steps, accel_val, decel_val, speed);
            printf("\n\r  ");
            okCmd = true;
        }
        else if(uart_buffer[1] == 'o')
        {
          if(uart_buffer[2] == 'v')
          {
            if(uart_buffer[3] == 'e')
            {
              //设置步数、加速度、减速度、最大速度
              if(uart_buffer[4] == ' ')
              {
                int i = 6;
                steps = atoi((char const *)uart_buffer+5);
                while((uart_buffer[i] != ' ') && (uart_buffer[i] != 13)) i++;
                i++;
                accel_val = atoi((char const *)uart_buffer+i);
                while((uart_buffer[i] != ' ') && (uart_buffer[i] != 13)) i++;
                i++;
                decel_val = atoi((char const *)uart_buffer+i);
                while((uart_buffer[i] != ' ') && (uart_buffer[i] != 13)) i++;
                i++;
                speed = atoi((char const *)uart_buffer+i);
                stepper_move_T(steps, accel_val, decel_val, speed);
                printf("\n\r  ");
                okCmd = true;
              }
            }
          }
        }
    }
    else if((uart_buffer[0] == 'a'))
    {
        //设置加速度
        if(uart_buffer[1] == ' ')
        {
          acc_temp = atoi((char const *)uart_buffer+2);
          if(acc_temp>=71 && acc_temp<=32000)
          {
            accel_val = acc_temp;
            printf("\n\r加速度:%.2frad/s^2",1.0*accel_val/100);
            okCmd = true;
          }
        }
    }
    else if(uart_buffer[0] == 'd')
    {
        //设置减速度
        if(uart_buffer[1] == ' ')
        {
            dec_temp = atoi((char const *)uart_buffer+2);
            if(dec_temp>=71 && dec_temp<=32000)
            {
                decel_val = dec_temp;
                printf("\n\r减速度:%.2frad/s^2",1.0*decel_val/100);
                okCmd = true;
            }
        }
    }
    else if(uart_buffer[0] == 's')
    {
        //设置最大速度
        if(uart_buffer[1] == ' ')
        {
            speed_temp = atoi((char const *)uart_buffer+2);
            if(speed_temp>=12 && speed_temp<=20000)
            {
                speed = speed_temp;
                printf("\n\r最大速度:%.2frad/s",1.0*speed/100);
                okCmd = true;
            }
        }
    }
    else if(uart_buffer[0] == '?')
    {
        //打印帮助命令
        ShowHelp();
        okCmd = true;
    }
    else if(uart_buffer[0] == 't')
    {
        //步进电机测试指令
        MSD_demo_run();
        okCmd = true;
    }
    //如果指令有无则打印帮助命令
    if(okCmd != true)
    {
        printf("\n\r 输入有误，请重新输入...");
        ShowHelp();
    }
    okCmd = false;

    Clear_buffer();

    if(status.running == true)
    {
        printf("\n\r电机正在运行...\r\n");

        while(status.running == true)
        {
            R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_MILLISECONDS);
        };

        printf("OK\n\r");
        ShowData(stepPosition, accel_val, decel_val, speed, steps);

    }
}

void MSD_demo_run(void)
{
    uint8_t step_cnt=0;
    int8_t step_num[10] = {2,2,-2,-2,2,2,-4,-4,4,20};
    for(int i=0;i<5;i++)
    {
                if(step_cnt==11)
                        step_cnt=0;
                while(status.running == TRUE)
                {
                        if(status.out_ena != TRUE)
                             break;
                };
                if(status.out_ena != TRUE)
                        break;
                else
                {
                        R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_SECONDS); //延时1秒
                        stepper_move_T(SPR*step_num[step_cnt], 32000, 32000, 3000);

                        ShowData(stepPosition, 32000, 32000, 3000, SPR*step_num[step_cnt]);
                        step_cnt++;
                }
    }
    step_cnt=0;
}

void ShowHelp(void)
{
    printf("\n\r——————————————野火步进电机梯形加减速演示程序——————————————");
    printf("\n\r输入命令：");
    printf("\n\r< ? >       -帮助菜单");
    printf("\n\ra[data]     -设置步进电机的加速度（范围：71—32000）单位为：0.01rad/s^2");
    printf("\n\rd[data]     -设置步进电机的减速度（范围：71—32000）单位为：0.01rad/s^2");
    printf("\n\rs[data]     -设置步进电机的最大速度（范围：12—3000）单位为：0.01rad/s");
    printf("\n\rm[data]     -以设定的步数移动步进电机（范围：-2147483647—2147483647，其中正数为顺时针，负数为逆时针）");

    printf("\n\rmove [steps] [accel] [decel] [speed]\r\n");

}

void ShowData(int position, int accel_val, int decel_val, int speed, int steps)
{
  printf("\n\r加速度:%.2frad/s^2",1.0*accel_val/100);
  printf("  减速度:%.2frad/s^2",1.0*decel_val/100);
  printf("  最大速度:%.2frad/s(%.2frpm)",1.0*speed/100,9.55*speed/100);
  printf("  移动步数:%d",steps);
  printf("\n\r电机当前位置: %d\r\n",position);
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
