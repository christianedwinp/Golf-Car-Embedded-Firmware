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
struct Stack ultrasonic = {{0,0,0,0,0,0,0,0},
													 {0,0,0,0,0,0,0,0},
													 {0,0,0,0,0,0,0,0},
													 {0,0,0,0,0,0,0,0},
													 {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}},
													 {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}},
													 {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}},
													 {false,false,false,false,false,false,false,false},
													 0}; 
byte INTERFACE_UART = 0, INTERFACE_TCI = 1, INTERFACE_OWU = 2, INTERFACE_SPI = 3;
byte P1_BL = 0, P2_BL = 1, P1_LO = 2, P2_LO = 3;
extern 	byte ultraMeasResult[34+3];

//USER PARAMETER : change according to your need
#define RECORD_EXPERIMENT 1
#define CONFIGURE_THRESHOLD_GAIN 0
#define GET_DISTANCE 1													 
short NEW_UART_ADDRESS = 1;
#define NUMBER_OBJ_DETECTED 1
double minDistLim = 0.1;
bool alwaysLongRangeMode = false;			
													 
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
	if(!configPGA460(INTERFACE_UART,115200,NEW_UART_ADDRESS,CHECK_ADDRESS_ON,ROOM_TEMPERATURE,SYS_DIAGNOSIS_ON,ECHO_DATA_DUMP_ON, &ultrasonic)){
		TEST_LED_PORT -> BRR = TEST_LED_2;
		printf("ERROR - PGA460 Initialization Unsuccesfull \r\n");
		return 0;
	}
		
	delay_ms(500);
	printf("all initialized  \r\n");
	if(RECORD_EXPERIMENT && GET_DISTANCE){
		printf("Addr,SR/LR,NumOfObject,Dist(m),Width(us),Peak(8bit) \r\n");
		printf("========================================================= \r\n");
	}	
	
	while (1)
	{
		//EXPERIMENT PROGRAM
		if(CONFIGURE_THRESHOLD_GAIN){
			for(int i=0; i < ultrasonic.size; i++){
				//Threshold Mapping
				autoThresholdRun(P1_BL, ultrasonic.address[0], 2, 7, 12, 2);//mode, uartIndex, noiseMargin, thrTimeIndex, thrPoints, loops, copyThr
				
				printf("Address %x - EED: ",ultrasonic.address[0]);
				runEchoDataDump(SHORT_DIST_MEASUREMENT, ultrasonic.address[0]); //run prset 1 
				for (int n = 0; n < 128; n++){
					byte echoDataDumpElement = pullEchoDataDump(n,ultrasonic.address[0]);
					printf("%d ",echoDataDumpElement);
				}
				printf("\r\n");
			}
		}
		
		//GET DISTANCE PROGRAM
		if(GET_DISTANCE){
			
			//Algorithm : detect short range, if no object detected use long range measurement, else no object or object too close
			for(int i=0; i < ultrasonic.size; i++){
					
					//Short range measurement, detect X object, from address Y
					ultrasonicCmd(SHORT_DIST_MEASUREMENT,NUMBER_OBJ_DETECTED,ultrasonic.address[i]);
					pullUltrasonicMeasResult(false,ultrasonic.address[i]);
				
					for(byte j=0; j < NUMBER_OBJ_DETECTED; j++){
						ultrasonic.distance[i][j] = printUltrasonicMeasResult(0 + (j * 3), &ultrasonic, i);
						ultrasonic.width[i][j] = printUltrasonicMeasResult(1 + (j * 3), &ultrasonic, i);
						ultrasonic.peak[i][j] = printUltrasonicMeasResult(2 + (j * 3), &ultrasonic, i);
						
						//calibration for measurement 30cm - 45cm
						if(ultrasonic.distance[i][j] >= 0.387 && ultrasonic.distance[i][j] < 0.464){
							ultrasonic.distance[i][j] = ultrasonic.distance[i][j] - ((0.4644-ultrasonic.distance[i][j]) * 0.3578 + 0.0044);
						}else if(ultrasonic.distance[i][j] >= 0.316 && ultrasonic.distance[i][j] < 0.387){
							ultrasonic.distance[i][j] = ultrasonic.distance[i][j] - ((ultrasonic.distance[i][j]-0.3168) * 0.4329 + 0.01682);
						}
						
						if((ultrasonic.distance[i][j]>minDistLim) && (ultrasonic.distance[i][j]<2.1)){
							if(RECORD_EXPERIMENT){
								printf("%x,1,%x,%f,%f,%f \r\n",ultrasonic.address[i], j+1, ultrasonic.distance[i][j],ultrasonic.width[i][j],ultrasonic.peak[i][j]);
							}else{
								printf("Address %x - SR - Obj %x - distance(m): %f - width(us): %f - peak(8bit): %f \r\n",ultrasonic.address[i], j+1, ultrasonic.distance[i][j],ultrasonic.width[i][j],ultrasonic.peak[i][j]);  
							}						
							ultrasonic.objectIsDetected[i] = true;
						}
					}
					
					//if failed to read short range, change to long range
					if(ultrasonic.objectIsDetected[i] == false || alwaysLongRangeMode == true){
						ultrasonicCmd(LONG_DIST_MEASUREMENT,NUMBER_OBJ_DETECTED,ultrasonic.address[i]);
						pullUltrasonicMeasResult(false,ultrasonic.address[i]);
						
						for(byte j=0; j < NUMBER_OBJ_DETECTED; j++){
							ultrasonic.distance[i][j] = printUltrasonicMeasResult(0 + (j * 3), &ultrasonic, i);
							ultrasonic.width[i][j] = printUltrasonicMeasResult(1 + (j * 3), &ultrasonic, i);
							ultrasonic.peak[i][j] = printUltrasonicMeasResult(2 + (j * 3), &ultrasonic, i);
							
							if((ultrasonic.distance[i][j]>minDistLim) && (ultrasonic.distance[i][j]<11.2)){
								if(RECORD_EXPERIMENT){
									printf("%x,2,%x,%f,%f,%f \r\n",ultrasonic.address[i], j+1, ultrasonic.distance[i][j],ultrasonic.width[i][j],ultrasonic.peak[i][j]);
								}else{
									printf("Address %x - LR - Obj %x - distance(m): %f - width(us): %f - peak(8bit): %f \r\n",ultrasonic.address[i], j+1, ultrasonic.distance[i][j],ultrasonic.width[i][j],ultrasonic.peak[i][j]);  
								}
								ultrasonic.objectIsDetected[i] = true;
							}else if(ultrasonic.distance[i][j] == 0){
								if(RECORD_EXPERIMENT){
									printf("%x,0,0,0,0,0 \r\n",ultrasonic.address[i]);
								}else{
									printf("Address %x - Error reading measurement result  \r\n",ultrasonic.address[i]);
								}						
							}else if(j == NUMBER_OBJ_DETECTED-1 &&  ultrasonic.objectIsDetected[i] == false){
								if(RECORD_EXPERIMENT){
									printf("%x,2, , , , \r\n",ultrasonic.address[i]);
								}else{
									printf("Address %x - No object!!!  \r\n", ultrasonic.address[i]);
								}	
							}
						}
					}
					
					//reset objectIsDetected variable to false
					ultrasonic.objectIsDetected[i] = false;
				}
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

