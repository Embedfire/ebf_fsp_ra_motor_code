#include "encoder.h"
#include "debug_uart/bsp_debug_uart.h"


uint8_t a,b = 0;
float num = 0;
uint32_t x = 1;
int flag = 1;
float speed1 = 0.0;

/* 初始化编码器 */
void initEncoder(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /* 打开定时器1 */

	err = R_GPT_Open(&encoder_ctrl, &encoder_cfg);
    assert(FSP_SUCCESS == err);
	
	err = R_GPT_Open(&count_ctrl, &count_cfg);
    assert(FSP_SUCCESS == err);
	
	/* 启用定时器1 */
	R_GPT_Enable(&encoder_ctrl);

    
    /* 启动PWM1定时器 */
    err = R_GPT_Start(&encoder_ctrl);
    assert(FSP_SUCCESS == err);
	
    err = R_GPT_Start(&count_ctrl);
    assert(FSP_SUCCESS == err);
	

}

/* 定时器1中断回调函数 */
void encoder_callback(timer_callback_args_t * p_args)
{
    switch(p_args->event)
    {
        /* 捕获事件 -- A沿触发 (A') 上升沿 */
        case TIMER_EVENT_CAPTURE_A:
            flag = -1;
            break;

        /* 捕获事件 -- B沿触发 */
        case TIMER_EVENT_CAPTURE_B:
            flag = 1;
           // 进行相关处理
            break;

        /* 定时周期结束事件 */
        case TIMER_EVENT_CYCLE_END:
            break;

        default:
            break;
    }
}

/* 定时器0中断回调函数 */
void count_callback(timer_callback_args_t * p_args)
{
	a ++;
    R_GPT_Stop(&encoder_ctrl);
    /* 定时周期结束事件 */
    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        x = encoder_ctrl.p_reg->GTCNT;
        speed1 = (float)( x * 50 / 4 );
		
		num += (float)x;
		
        R_GPT_CounterSet(&encoder_ctrl, 0);
    }
	
	if(a == 50)
	{	
		b++;
		if(flag == 1)
			printf("\r\n当前步进电机方向为 顺时针\r\n");
		else
			printf("当前步进电机方向为 逆时针\r\n");
		
		printf("当前步进电机转速为 %.2f 转每秒\r\n",(speed1/1000));
		
		printf("目前累计圈数为 %.2f 圈\r\n",num/1000/4);
		
		a = 0;
	}
	//R_BSP_SoftwareDelay(500, BSP_DELAY_UNITS_MILLISECONDS);

    R_GPT_Start(&encoder_ctrl);
}