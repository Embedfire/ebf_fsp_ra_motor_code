#include "bsp_encoder.h"
#include "led/bsp_led.h"
#include "motor_dc_brush/motor_dc_brush.h"

uint32_t x = 1;
int flag = 1;
float speed1 = 0.0;

/* 初始化编码器 */
void initEncoder(void)
{
    fsp_err_t err = FSP_SUCCESS;
    /* 打开定时器1 */
    err = R_GPT_Open(&g_timer1_ctrl, &g_timer1_cfg);
    assert(FSP_SUCCESS == err);
	
	err = R_GPT_Open(&g_timer0_ctrl, &g_timer0_cfg);
    assert(FSP_SUCCESS == err);
	
	/* 启用定时器1 */
    R_GPT_Enable(&g_timer1_ctrl);
    
    /* 启动PWM1定时器 */
    err = R_GPT_Start(&g_timer1_ctrl);
    assert(FSP_SUCCESS == err);
	
	err = R_GPT_Start(&g_timer0_ctrl);
    assert(FSP_SUCCESS == err);
	
}

/* 定时器1中断回调函数 */
void time1_callback(timer_callback_args_t * p_args)		
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
            /* 定时周期结束时进行一些操作，这里示例是LED1状态翻转 */
			LED1_TOGGLE;
            break;

        default:
            break;
    }
    

}


/* 定时器0中断回调函数 */
void time0_callback(timer_callback_args_t * p_args)
{
	R_GPT_Stop(&g_timer1_ctrl);
    /* 定时周期结束事件 */
    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
		x = g_timer1_ctrl.p_reg->GTCNT;
		dc_motor_s.now_speed = (float)((x * 20 * 60 / 4.0 / 960.0) * flag);  
		R_GPT_CounterSet(&g_timer1_ctrl, 0);
    }
	R_GPT_Start(&g_timer1_ctrl);
}




