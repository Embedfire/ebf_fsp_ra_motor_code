#include "bsp_gpt_encoder.h"
#include "stdio.h"


/*  当前选用输入捕获引脚 GTIOC7A P603 */


/* GPT初始化函数 */
void Encoder_GPT1_Init(void)
{
    /* 初始化 GPT 模块 */
    R_GPT_Open(&g_timer1_encoder_ctrl, &g_timer1_encoder_cfg);
    
    /* 使能输入捕获 */
	R_GPT_Enable(&g_timer1_encoder_ctrl);

    /* 启动 GPT 定时器 */
    R_GPT_Start(&g_timer1_encoder_ctrl);
}
