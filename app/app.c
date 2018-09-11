#include "bsp.h"
#include "bsp_delay.h"
#include "bsp_io.h"
#include "bsp_timer.h"
#include "bsp_can.h"
#include "bsp_encoder.h"
#include "bsp_eps.h"
#include "bsp_usart.h"
#include "bsp_ultrasonic.h"
#include "bsp_time.h"
#include "pin_configuration.h"
#include "PGA460_USSC.h"
#include "stack.h"
#include <math.h>

enum MCO_DEBUG_MODE{NONE,SYSCLK,HSI,HSE,PLLCLK_DIV2};

extern volatile u32 Millis;
extern short PWMPeriodCnt;
extern 	byte ultraMeasResult[34+3]; 
RCC_ClocksTypeDef rcc;
uint8_t regaddrread = 0x1f,regaddrwr = 0x02;

byte INTERFACE_UART = 0; 
byte INTERFACE_TCI =	1;
byte INTERFACE_OWU =	2;
byte INTERFACE_SPI =	3;
short NEW_UART_ADDRESS = 6;
#define CHECK_ADDRESS_ON		1
#define CHECK_ADDRESS_OFF		0
#define SYS_DIAGNOSIS_ON 		1
#define SYS_DIAGNOSIS_OFF 	0
#define ECHO_DATA_DUMP_ON 	1
#define ECHO_DATA_DUMP_OFF	0

struct Stack temperatureReading = {{0,0,0,0,0,0,0,0},0}; 

unsigned char UltrSendflag = 0x00;
short IntervelCnt[2];
double objDist[8] ,objWidth[8],TempDis2,TempWidth2 ;
uint16_t TempDis=0,TempWidth=0;
byte objectDetected[8] = {false,false,false,false,false,false,false,false};

/**@ ONLY FOR DEBUG
 *   This func is used to configure MCO which can help debug clock
 *   The selected Clock will output to PA.8 (Encoder#1+)
 */
void MCO(enum MCO_DEBUG_MODE mode)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //this function is only valid for STM32F0x
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //this function is only valid for STM32F0x
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//this function is only valid for STM32F0x
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);
	//RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1);  // HSE=8M --> PA.8
	//RCC_MCO1Config(RCC_MCO1Source_PLLCLK,RCC_MCO1Div_4);  // PLL=168M, 168/4=42M -->PA.8
	
	// pick one of the clocks to spew
	switch(mode){
		case NONE:
			RCC_MCOConfig(RCC_MCO_NoClock); 
			break;
		case SYSCLK:
			RCC_MCOConfig(RCC_MCO_SYSCLK); // Put on MCO pin the: System clock selected
			break;
		case HSI:
			RCC_MCOConfig(RCC_MCO_HSI); // Put on MCO pin the: freq. of high-speed internal 16 MHz RC oscillator
			break;
		case HSE:
			RCC_MCOConfig(RCC_MCO_HSE); // Put on MCO pin the: freq. of external crystal i.e 4-26 MHz
			break;
		case PLLCLK_DIV2:
			RCC_MCOConfig(RCC_MCO_PLLCLK_Div2);
			break;
		default:
			RCC_MCOConfig(RCC_MCO_NoClock); // Put on MCO pin the: System clock selected
			break;
	};
}


// Determine the speed of sound by temperature sensor reading inside PGA460Q1
double speedSoundByTemp(double temp){
	if(temp >= 0){
		return temp*0.6+331;
	}else{
		return temp*(-0.6)+331;
	}
}

int main(void)
{
	RCC_GetClocksFreq(&rcc);
	Systick_Init();
	BSP_Init();
	BSP_DelayInit();	         		// Init delpy parameters
	BSP_LEDTestInit();       	    // Init LED on board 
	BSP_UsartInit(115200);     		// Init USART1 for debug, baudrate=115200
	//MCO(NONE);                  // Debug system clock, use it only when you want to check clock
	
	printf("init PGA460 \r\n");
	//TEST_LED_PORT -> BRR = TEST_LED_1;

	Stack_Init(&temperatureReading);
	if(!configPGA460(INTERFACE_UART,19200,NEW_UART_ADDRESS,CHECK_ADDRESS_ON,SYS_DIAGNOSIS_ON,ECHO_DATA_DUMP_ON, &temperatureReading)){
		return 0;
	}
	double avgTemperature = Stack_Avg(&temperatureReading); 
	double speedSound = speedSoundByTemp(avgTemperature);
	double digitalDelay = 0.00005 * speedSound;
	
	delay_ms(500);
	printf("all initialized  \r\n");
	
	while (1)
	{
		//ULTRASONIC ROUTINE

		//Short range measurement, detect 1 object, from address X
//		ultrasonicCmd(0,1,uartAddrUpdate);
//		//interval between burst for short range 
//		delay_ms(20);
//		// Pull Ultrasonic Measurement Result
//		pullUltrasonicMeasResult(false,uartAddrUpdate);      
//		TempDis = (ultraMeasResult[1]<<8) + ultraMeasResult[2];
//		TempWidth = ultraMeasResult[3];
//		TempDis2 = (TempDis/2*0.000001*speedSound) - digitalDelay;
//		TempWidth2 = TempWidth * 16;
//		if((TempDis2>0.15)&(TempDis2<1.2)){
//			objectDetected[uartAddrUpdate] = true;
//		}
//		
//		//if failed to read short range, change to long range
//		if(objectDetected[uartAddrUpdate] == false){
//			ultrasonicCmd(1,1,uartAddrUpdate);
//			//interval between burst for long range 
//			delay_ms(35); // maximum record length is 65ms, so delay within that margin
//			pullUltrasonicMeasResult(false,uartAddrUpdate);
//			TempDis = (ultraMeasResult[1]<<8) + ultraMeasResult[2];
//			TempWidth = ultraMeasResult[3];
//			TempDis2 = (TempDis/2*0.000001*speedSound) - digitalDelay;
//			TempWidth2= TempWidth * 16;

//			if((TempDis2<11.2)&&(TempDis2>0))
//			{
//				objectDetected[uartAddrUpdate] = true;
//				
//			}else if(TempDis2 == 0)
//			{
//			}else{
//				printf("No object!!!\n");
//			}
//		}

//		//just value final output objDist and objWidth once when short distance detect fail.
//		if(objectDetected[uartAddrUpdate] == false){
//			printf("no project or ultrasonic error\n");
//		}else{
//			objDist[uartAddrUpdate] = TempDis2;
//			objWidth[uartAddrUpdate] = TempWidth2;
//		}
//		
//		objectDetected[uartAddrUpdate] =false;
	}
}


void SysTick_Handler()
{
	// time counter
	Millis++;
//	IntervelCnt[1]++;
}

void TIM6_IRQHandler()
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update) == SET ){}//-11.2us
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update); 
	TIM_ClearFlag(TIM6,TIM_FLAG_Update);
}

