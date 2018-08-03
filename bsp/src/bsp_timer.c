#include "bsp_timer.h"
#include "bsp_io.h"
#include "pin_configuration.h"

volatile uint8_t gTimerFlag = 0;
short PWMPeriodCnt;
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
	
	RCC_APB2PeriphClockCmd(ENC_TIMER_CLK, ENABLE);     		//使能TIM1时钟
	
	TIM_TimeBaseInitStructure.TIM_Period = arr; 										//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200-1;  		    			//定时器分频, 每次基数72Mhz/7200=10Khz
	TIM_TimeBaseInitStructure.TIM_CounterMode= TIM_CounterMode_Up; 	//向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 

	TIM_TimeBaseInit(ENC_TIMER,&TIM_TimeBaseInitStructure);		//初始化TIM1
	TIM_ITConfig(ENC_TIMER,TIM_IT_Update,ENABLE); 						//允许定时器1更新中断
	TIM_Cmd(ENC_TIMER,ENABLE); 																//使能定时器1

	NVIC_InitStructure.NVIC_IRQChannel=TIM1_UP_IRQn; 							//定时器1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; 		//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; 					//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


void TIM1_UP_IRQHandler(void)
{
	
	if(TIM_GetITStatus(ENC_TIMER,TIM_IT_Update) == SET ) 
	{
		gTimerFlag = 1;
	}
	
	TIM_ClearITPendingBit(ENC_TIMER,TIM_IT_Update);  			
}


void BSP_Timer8PWM()
{
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure_TimerInterrupt;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//GPIO_Mode_Out_PP
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	TIM_TimeBaseStructure.TIM_Prescaler = 1; // clk = 72M/(1 + 1) = 36Mhz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_Period = 749; // freq = 36000 khz / 750 = 48k hz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 375; // 50% * TIM_Period value
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
  TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	
	NVIC_InitStructure_TimerInterrupt.NVIC_IRQChannel = TIM8_UP_IRQn;
  NVIC_InitStructure_TimerInterrupt.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure_TimerInterrupt.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure_TimerInterrupt.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure_TimerInterrupt);
	
  TIM_Cmd(TIM8, ENABLE);
}

void TIM8_UP_IRQHandler()
{
	if(TIM_GetITStatus(TIM8,TIM_IT_Update) == SET ) 
	{
		PWMPeriodCnt++;
		TIM_ClearITPendingBit(TIM8,TIM_IT_Update); 
		if(PWMPeriodCnt>=8)
		{
			PWMPeriodCnt=0;
			TIM_Cmd(TIM8, DISABLE);
		}
	}

}

