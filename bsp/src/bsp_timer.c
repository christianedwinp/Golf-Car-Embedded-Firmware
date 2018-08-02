#include "bsp_timer.h"
#include "bsp_io.h"
#include "pin_configuration.h"

volatile uint8_t gTimerFlag = 0;

/**
 * TIM2 72Mhz
 * 
 **/
void BSP_TimerInit(uint16_t freq)
{
	uint16_t arr = 500 -1;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	if(freq < 1000)
	{
		arr = 10000/(freq)-1;
	}
	
	RCC_APB1PeriphClockCmd(ENC_TIMER_CLK, ENABLE);     		//使能TIM1时钟
	
	TIM_TimeBaseInitStructure.TIM_Period = arr; 					//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200-1;  		    	//定时器分频, 每次基数72Mhz/7200=10Khz
	TIM_TimeBaseInitStructure.TIM_CounterMode= TIM_CounterMode_Up; 	//向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 

	TIM_TimeBaseInit(ENC_TIMER,&TIM_TimeBaseInitStructure);				//初始化TIM2
	TIM_ITConfig(ENC_TIMER,TIM_IT_Update,ENABLE); 						//允许定时器2更新中断
	TIM_Cmd(ENC_TIMER,ENABLE); 											//使能定时器2

	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; 					//定时器2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x03; 		//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; 			//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(ENC_TIMER,TIM_IT_Update) ==SET ) 
	{
		gTimerFlag = 1;
	}
	TIM_ClearITPendingBit(ENC_TIMER,TIM_IT_Update);  			
}



