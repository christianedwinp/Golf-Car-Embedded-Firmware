#include "bsp_encoder.h"
#include "pin_configuration.h"

/**       STM32F103VET6
 *             		TIMx        CH1     CH2
 * BACK_LEFT    	TIM2        PA15    PB3
 * BACK_RIGHT   	TIM3        PA6  		PA7
 * FRONT_LEFT			TIM4				PB6			PB7
 * FRONT_RIGHT		TIM5				PA0			PA1
 */

#define CR1_DIR_MASK    (0x0010)
const uint16_t gReload = 0x3fff;
const uint16_t gGapLow = 0x100;
const uint16_t gGapHigh = gReload * 2 - gGapLow;
uint32_t gLast[4] = {gReload, gReload, gReload, gReload};

TIM_TypeDef* TIMs[4] = {BACK_LEFT_TIMER,BACK_RIGHT_TIMER, FRONT_LEFT_TIMER, FRONT_RIGHT_TIMER}; //TIM2,3,4,5

int16_t gEncoder[4] = {1, 2, 3, 4};


void BSP_EncoderInit(void)
{
	GPIO_InitTypeDef         GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef        TIM_ICInitStructure;

	
	//GPIO INITIALIZATION
	RCC_APB1PeriphClockCmd(BACK_LEFT_TIMER_CLK | BACK_RIGHT_TIMER_CLK | FRONT_LEFT_TIMER_CLK | FRONT_RIGHT_TIMER_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | BACK_LEFT_CH1_CLK | BACK_LEFT_CH2_CLK | BACK_RIGHT_CLK | FRONT_LEFT_CLK | FRONT_RIGHT_CLK, ENABLE);
	
	
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);//部分重映射
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
	//GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);

	
	GPIO_InitStructure.GPIO_Pin = BACK_LEFT_CH1;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(BACK_LEFT_CH1_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = BACK_LEFT_CH2;
	GPIO_Init(BACK_LEFT_CH2_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = BACK_RIGHT_CH1 | BACK_RIGHT_CH2;
	GPIO_Init(BACK_RIGHT_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = FRONT_LEFT_CH1 | FRONT_LEFT_CH2;
	GPIO_Init(FRONT_LEFT_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = FRONT_RIGHT_CH1 | FRONT_RIGHT_CH2;
	GPIO_Init(FRONT_RIGHT_PORT, &GPIO_InitStructure);
	
	//ENCODER TIMER INITIALIZATION
	// TIM2/3/4/5 72Mhz 
	TIM_DeInit(BACK_LEFT_TIMER);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = gReload * 2; 						//下一个更新事件自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler = 0;        						//设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    	//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(BACK_LEFT_TIMER, &TIM_TimeBaseStructure); 
	
	TIM_DeInit(BACK_RIGHT_TIMER);
	TIM_TimeBaseInit(BACK_RIGHT_TIMER, &TIM_TimeBaseStructure); 
	
	TIM_DeInit(FRONT_LEFT_TIMER);
	TIM_TimeBaseInit(FRONT_LEFT_TIMER, &TIM_TimeBaseStructure); 
	
	TIM_DeInit(FRONT_RIGHT_TIMER);
	TIM_TimeBaseInit(FRONT_RIGHT_TIMER, &TIM_TimeBaseStructure); 
	

	//使用编码器模式3，上升下降都计数
	TIM_EncoderInterfaceConfig(BACK_LEFT_TIMER, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);
	TIM_EncoderInterfaceConfig(BACK_RIGHT_TIMER, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);
	TIM_EncoderInterfaceConfig(FRONT_LEFT_TIMER, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);
	TIM_EncoderInterfaceConfig(FRONT_RIGHT_TIMER, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);
	
  TIM_ICStructInit(&TIM_ICInitStructure);//将结构体中的内容缺省输入
  TIM_ICInitStructure.TIM_ICFilter = 4;  //选择输入比较滤波器 
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInit(BACK_LEFT_TIMER, &TIM_ICInitStructure);
	TIM_ICInit(BACK_RIGHT_TIMER, &TIM_ICInitStructure);
	TIM_ICInit(FRONT_LEFT_TIMER, &TIM_ICInitStructure);
	TIM_ICInit(FRONT_RIGHT_TIMER, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInit(BACK_LEFT_TIMER, &TIM_ICInitStructure);
	TIM_ICInit(BACK_RIGHT_TIMER, &TIM_ICInitStructure);
	TIM_ICInit(FRONT_LEFT_TIMER, &TIM_ICInitStructure);
	TIM_ICInit(FRONT_RIGHT_TIMER, &TIM_ICInitStructure);
		
	TIM_ClearFlag(BACK_LEFT_TIMER, TIM_FLAG_Update);  
	TIM_ClearFlag(BACK_RIGHT_TIMER, TIM_FLAG_Update);
	TIM_ClearFlag(FRONT_LEFT_TIMER, TIM_FLAG_Update);  
	TIM_ClearFlag(FRONT_RIGHT_TIMER, TIM_FLAG_Update);    
	
	TIM_Cmd(BACK_LEFT_TIMER, ENABLE); 
	TIM_Cmd(BACK_RIGHT_TIMER, ENABLE);
	TIM_Cmd(FRONT_LEFT_TIMER, ENABLE); 
	TIM_Cmd(FRONT_RIGHT_TIMER, ENABLE); 
}


/**
 * @breif read encoder value and clear register
 */
void BSP_EncoderRead(void)
{
	uint8_t i = 0;
	
	for (i = 0; i < 4; i++)
	{
		gEncoder[i] = (TIMs[i]->CNT - gLast[i]);
		if (TIMs[i] ->CNT > gGapHigh || TIMs[i]->CNT < gGapLow  )
		{
			TIMs[i]->CNT = gReload;
			gLast[i] = gReload;
		}
		else
		{
			gLast[i] = TIMs[i]->CNT;
		}

	}
	//IF YOU READ NEGATIVE THEN ADD NEGATIVE SYMBOL INFRONT THE gEncoder[x]
	//e.g : gEncoder[2] = -gEncoder[2];
	gEncoder[0] = gEncoder[0]; 
	gEncoder[1] = gEncoder[1]; 
	gEncoder[2] = gEncoder[2];
	gEncoder[3] = gEncoder[3];
}
