#include "stm32f10x.h"
#include "system_stm32f10x.h"
#include "bsp_time.h"

volatile u32 Millis;

void Systick_Init(void)
{
 	if (SysTick_Config (72000)) //1ms per interrupt
	while (1);

	//set systick interrupt priority
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);    //4 bits for preemp priority 0 bit for sub priority
	//NVIC_SetPriority(SysTick_IRQn, 0);//i want to make sure systick has highest priority amount all other interrupts
	
	Millis = 0;//reset Millis
}



uint32_t millis()
{
	return Millis;
}

long msdelay;
uint32_t micros()
{
	
	return Millis*1000 + 1000 - SysTick -> VAL/72;
}

void delay_ms(u32 nTime)
{
	u32 curTime = Millis;
	msdelay =10;
	while( msdelay > 0)
	{
		msdelay = (nTime-(Millis-curTime));
	}
}
long usdelay=10;
void delay_us(u32 nTime)
{
	long curTime = micros();
	usdelay=10;
	while(usdelay > 0)
	{
		usdelay = 	nTime-(micros()-curTime);
	}
}


