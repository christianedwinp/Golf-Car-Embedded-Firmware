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
#include <math.h>

enum MCO_DEBUG_MODE{NONE,SYSCLK,HSI,HSE,PLLCLK_DIV2};

extern volatile u32 Millis;
RCC_ClocksTypeDef rcc;
uint8_t regaddrread = 0x1f,regaddrwr = 0x02;


short NEW_UART_ADDRESS = 6;
#define NUMBER_OBJ_DETECTED 1
double minDistLim = 0.15;
bool alwaysLongRangeMode = false;
extern 	byte ultraMeasResult[34+3];
byte INTERFACE_UART = 0, INTERFACE_TCI = 1, INTERFACE_OWU = 2, INTERFACE_SPI = 3;
struct Stack ultrasonic = {{0,0,0,0,0,0,0,0},
													 {0,0,0,0,0,0,0,0},
													 {0,0,0,0,0,0,0,0},
													 {0,0,0,0,0,0,0,0},
													 {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}},
													 {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}},
													 {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}},
													 {false,false,false,false,false,false,false,false},
													 0}; 

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
	Stack_Init(&ultrasonic);
	if(!configPGA460(INTERFACE_UART,19200,NEW_UART_ADDRESS,CHECK_ADDRESS_ON,SYS_DIAGNOSIS_ON,ECHO_DATA_DUMP_ON, &ultrasonic)){
		printf("Initialization Unsuccesfull \r\n");
		return 0;
	}
		
	delay_ms(500);
	printf("all initialized  \r\n");
	
	while (1)
	{
		//ULTRASONIC ROUTINE
		//Algorithm : detect short range, if no object detected use long range measurement, else no object or object too close
		for(int i=0; i < ultrasonic.size; i++){
			
			//Short range measurement, detect X object, from address Y
			ultrasonicCmd(SHORT_DIST_MEASUREMENT,NUMBER_OBJ_DETECTED,ultrasonic.address[i]);
			//interval between burst cycle for short range 
			delay_ms(20);
			//Pull Ultrasonic Measurement Result
			pullUltrasonicMeasResult(false,ultrasonic.address[i]);
			
			for(byte j=0; j < NUMBER_OBJ_DETECTED; j++){
				ultrasonic.distance[i][j] = printUltrasonicMeasResult(0 + (j * 3), &ultrasonic, i);
				ultrasonic.width[i][j] = printUltrasonicMeasResult(1 + (j * 3), &ultrasonic, i);
				ultrasonic.peak[i][j] = printUltrasonicMeasResult(2 + (j * 3), &ultrasonic, i);
				
				if((ultrasonic.distance[i][j]>minDistLim)&(ultrasonic.distance[i][j]<11.2)){
					printf("SR Obj %x distance (m) : %f \r\n", j+1, ultrasonic.distance[i][j]);  
					ultrasonic.objectIsDetected[i] = true;
				}
				
				if(j == NUMBER_OBJ_DETECTED-1 && ultrasonic.objectIsDetected[i] == false){
					printf("change to LR \r\n");
				}
			}
			
			//if failed to read short range, change to long range
			if(ultrasonic.objectIsDetected[i] == false || alwaysLongRangeMode == true){
				ultrasonicCmd(LONG_DIST_MEASUREMENT,NUMBER_OBJ_DETECTED,ultrasonic.address[i]);
				//interval between burst for long range 
				delay_ms(35); // maximum record length is 65ms, so delay within that margin
				pullUltrasonicMeasResult(false,ultrasonic.address[i]);
				
				for(byte j=0; j < NUMBER_OBJ_DETECTED; j++){
					ultrasonic.distance[i][j] = printUltrasonicMeasResult(0 + (j * 3), &ultrasonic, i);
					ultrasonic.width[i][j] = printUltrasonicMeasResult(1 + (j * 3), &ultrasonic, i);
					ultrasonic.peak[i][j] = printUltrasonicMeasResult(2 + (j * 3), &ultrasonic, i);
					
					if((ultrasonic.distance[i][j]>minDistLim)&(ultrasonic.distance[i][j]<11.2)){
						printf("LR Obj %x distance (m) : %f \r\n", j+1, ultrasonic.distance[i][j]);  
						ultrasonic.objectIsDetected[i] = true;
					}else if(ultrasonic.distance[i][j] == 0){
						printf("Error reading measurement result  \r\n");
					}else if(j == NUMBER_OBJ_DETECTED-1 &&  ultrasonic.objectIsDetected[i] == false){
						printf("No object!!!  \r\n");
					}
				}
			}
			
			//reset objectIsDetected variable to false
			ultrasonic.objectIsDetected[i] = false;
		}
	}
}


void SysTick_Handler()
{
	// time counter
	Millis++;
}

void TIM6_IRQHandler()
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update) == SET ){}//-11.2us
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update); 
	TIM_ClearFlag(TIM6,TIM_FLAG_Update);
}

