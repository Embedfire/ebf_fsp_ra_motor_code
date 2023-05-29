#include "bsp_dht11.h"
bsp_io_level_t level_1;

void DHT11_IO_Init(void)
{

    R_IOPORT_Open (&g_ioport_ctrl, g_ioport.p_cfg);

}


void DHT11_RST (void)
{                      //DHT11端口复位，发出起始信号（IO发送）
    R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_13_PIN_13, IOPORT_CFG_PORT_DIRECTION_OUTPUT);                         //端口为输出
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_13_PIN_13, BSP_IO_LEVEL_LOW);//使总线为低电平
    R_BSP_SoftwareDelay(20,BSP_DELAY_UNITS_MILLISECONDS);                           //拉低至少18ms
    R_IOPORT_PinWrite(&g_ioport_ctrl, BSP_IO_PORT_13_PIN_13, BSP_IO_LEVEL_HIGH);   //使总线为低电平
    R_BSP_SoftwareDelay(13,BSP_DELAY_UNITS_MICROSECONDS);                           //拉高13us
}

/*端口输入时，配置为上拉输入或者浮空输入，因为外接上拉电阻，所以默认为高电平
如果DHT11的数据线输入为高电平，且 retry 小于100，则将 retry 加1，并延时1微秒
重复这个过程直到 retry 大于等于100 或者DHT11的数据线输入变成低电平。
如果 retry 大于等于100，表示检测失败，返回1；否则，将 retry 重置为0。*/
uint8_t DHT11_Check(void)
{
    uint8_t retry=0;         //定义临时变量
    R_IOPORT_PinCfg(&g_ioport_ctrl, BSP_IO_PORT_13_PIN_13, IOPORT_CFG_PORT_DIRECTION_INPUT);      //IO到输入状态
    bsp_io_level_t level_state;
    /*while ((R_BSP_PinRead(BSP_IO_PORT_13_PIN_13) == 0) && retry<100)  //DHT11拉低后会再次拉高40~80us
         {
           retry++;
            R_BSP_SoftwareDelay(1,BSP_DELAY_UNITS_MICROSECONDS);
         }
         if(retry>=100)return 1;
         else  retry=0        */
    do{

         R_IOPORT_PinRead(&g_ioport_ctrl, BSP_IO_PORT_13_PIN_13, &level_state);
         retry++;
         R_BSP_SoftwareDelay(1,BSP_DELAY_UNITS_MICROSECONDS);

      }while (level_state && retry<100);                        //DHT11会拉低40~80us


       if(retry>=100)return 1;
       else retry=0;
//如果DHT11的数据线输入为低电平，且 retry 小于100，则将 retry 加1，并延时1微秒，重复这个过程直到 retry 大于等于100 或者DHT11的数据线输入变成高电平。如果 retry 大于等于100，表示检测失败，返回1；否则，返回0，表示检测成功。
    do{
        R_IOPORT_PinRead(&g_ioport_ctrl, BSP_IO_PORT_13_PIN_13, &level_state);
        retry++;
        R_BSP_SoftwareDelay(1,BSP_DELAY_UNITS_MICROSECONDS);

      }while(!level_state&&retry<100);

    if(retry>=100)return 1;
    return 0;

    /*while ((R_BSP_PinRead(BSP_IO_PORT_13_PIN_13) == 0) && retry<100)  //DHT11拉低后会再次拉高40~80us
     {
       retry++;
        R_BSP_SoftwareDelay(1,BSP_DELAY_UNITS_MICROSECONDS);
     }
     if(retry>=100)return 1;
     return 0;*/
}


uint8_t DHT11_Init (void)                         //DHT11初始化
{
    DHT11_RST();                                //DHT11端口复位，发出起始信号
    return DHT11_Check();                     //等待DHT11回应
}



uint8_t DHT11_Read_Bit(void)
{
    uint8_t retry = 0;
    while((R_BSP_PinRead(BSP_IO_PORT_13_PIN_13) == 1) && retry < 100) //等待变为低电平
    {
        retry++;
        R_BSP_SoftwareDelay(1,BSP_DELAY_UNITS_MICROSECONDS);       //1us
    }
    retry = 0;
    while((R_BSP_PinRead(BSP_IO_PORT_13_PIN_13) == 0) && retry < 100) //等待变高电平
    {
        retry++;
        R_BSP_SoftwareDelay(1,BSP_DELAY_UNITS_MICROSECONDS);       //1us
    }
    R_BSP_SoftwareDelay(40,BSP_DELAY_UNITS_MICROSECONDS);//         等待40us
    if(R_BSP_PinRead(BSP_IO_PORT_13_PIN_13) == 1)       //用于判断高低电平，即数据1或0
        return 1;
    else
        return 0;
}



uint8_t DHT11_Read_Byte(void)
{
    uint8_t i, dat;
    dat = 0;
    for (i = 0; i < 8; i++)
    {
        dat <<= 1;                  //左移运算符,dat左移1位
        dat |= DHT11_Read_Bit();    //"|"表示按位或等于
    }
    return dat;
}

uint8_t DHT11_Read_Data(uint8_t *temp, uint8_t *humi)
{
    uint8_t buf[5];
    uint8_t i;
    DHT11_RST();                        //DHT11端口复位，发出起始信号
    if(DHT11_Check() == 0)              //等待DHT11回应，0为成功回应
    {
        for(i = 0; i < 5; i++)          //读取40位数据
        {
            buf[i] = DHT11_Read_Byte(); //读出数据
        }
        if((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])   //数据校验
        {
            *humi=buf[0];
            *(humi + 1)=buf[1];
            *temp=buf[2];
            *(temp + 1)=buf[3];
        }
    }
    else return 1;
    return 0;
}












