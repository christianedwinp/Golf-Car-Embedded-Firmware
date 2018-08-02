#include "bsp_delay.h"


////////////////////////////////////////////////////////////////////////////////// 

static uint8_t  fac_us=0;							//us延时倍乘数			   
static uint16_t fac_ms=0;							//ms延时倍乘数,在os下,代表每个节拍的ms数
			   
//Initialize the delay function
//This function initializes the clock beat of the OS when using the OS.
//SYSTICK clock is fixed to 1/8 of AHB clock
//SYSCLK: system clock frequency
void BSP_DelayInit(void)
{	
 	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	fac_us=SystemCoreClock/8000000;						//represents the number of systick clocks required for each uS
	fac_ms=(uint16_t)fac_us*1000;							//represents the number of systick clocks required for each mS
}								    


//Delay micro second
//param : number of uS to delay, max 798915us i.e from 2^24/fac_us @ fac_us=21
void BSP_DelayUs(uint32_t nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; 					//time loading	  		 
	SysTick->VAL=0x00;        					//clear counter
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//start counting down	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//wait till time arrive   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//close counter
	SysTick->VAL =0X00;      					 //empty counter	 
}

//Delay milli second
//param : number of mS to delay, max 1864mS under 72Mhz sysclk
void BSP_DelayMs(uint16_t nms)
{	 	 
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;				//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;							//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;       					//清空计数器	  	  
}  
			 

