#include "bsp_ultrasonic.h"
#include "pin_configuration.h"
#include "bsp_time.h"
#include <stdio.h>

uint32_t start1,finish1, delta1, start2,finish2, delta2;
int rising1 = 1,rising2 = 1, transmitFlag1 = 0, transmitFlag2 = 0;

void BSP_UltrasonicInit(){
	initMux();
	switchOnMux(3);
	initEcho1();	
	initEcho2();	
	initTrig();
	initVoltageCompare();
}

void initMux(){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(MUXCLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = MUX1_ENABLE | MUX1_CHANNEL_SELECT_A | MUX1_CHANNEL_SELECT_B | MUX2_ENABLE | MUX2_CHANNEL_SELECT_A | MUX2_CHANNEL_SELECT_B  ; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MUXPORT, &GPIO_InitStructure);
}

void selectChannel_Mux1(int channel){
	switch(channel){
		case 1://00
			MUXPORT->BRR = MUX1_CHANNEL_SELECT_A;
			MUXPORT->BRR = MUX1_CHANNEL_SELECT_B;			
			break;
		case 2://10
			MUXPORT->BSRR = MUX1_CHANNEL_SELECT_A;
			MUXPORT->BRR = MUX1_CHANNEL_SELECT_B;
			break;
		case 3://01
			MUXPORT->BRR = MUX1_CHANNEL_SELECT_A;
			MUXPORT->BSRR 	= MUX1_CHANNEL_SELECT_B;
			break;
		case 4://11
			MUXPORT->BSRR = MUX1_CHANNEL_SELECT_A;
			MUXPORT->BSRR	= MUX1_CHANNEL_SELECT_B;
			break;
		default:
			printf("Channel number outbound, only input 1/2/3/4   \r 		\n");
	}
}

void selectChannel_Mux2(int channel){
	switch(channel){
		case 1:
			MUXPORT->BRR = MUX2_CHANNEL_SELECT_A;
			MUXPORT->BRR = MUX2_CHANNEL_SELECT_B;			
			break;
		case 2:
			MUXPORT->BSRR = MUX2_CHANNEL_SELECT_A;
			MUXPORT->BRR = MUX2_CHANNEL_SELECT_B;
			break;
		case 3:
			MUXPORT->BRR = MUX2_CHANNEL_SELECT_A;
			MUXPORT->BSRR 	= MUX2_CHANNEL_SELECT_B;
			break;
		case 4:
			MUXPORT->BSRR = MUX2_CHANNEL_SELECT_A;
			MUXPORT->BSRR	= MUX2_CHANNEL_SELECT_B;
			break;
		default:
			printf("Channel number outbound, only input 1/2/3/4   \r 		\n");
	}
}

void switchOffMux(int mux){
	switch(mux){
		case 1:
		MUXPORT->BSRR  = MUX1_ENABLE;		
		break;
		case 2:
		MUXPORT->BSRR  = MUX2_ENABLE;
		break;
		case 3:
		MUXPORT->BSRR  = MUX1_ENABLE;	
		MUXPORT->BSRR  = MUX2_ENABLE;
		break;
		default:
		printf("Multiplexer number outbound, only input 1/2/3   \r 		\n");
	}
}
void switchOnMux(int mux){
	switch(mux){
		case 1:
		MUXPORT->BRR  = MUX1_ENABLE;		
		break;
		case 2:
		MUXPORT->BRR  = MUX2_ENABLE;
		break;
		case 3:
		MUXPORT->BRR  = MUX1_ENABLE;	
		MUXPORT->BRR  = MUX2_ENABLE;
		break;
		default:
		printf("Multiplexer number outbound, only input 1/2/3   \r 		\n");
	}
}

void initTrig(){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(ULTRASONIC_TRIGCLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_TRIG; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ULTRASONIC_TRIGPORT, &GPIO_InitStructure);
}

void initEcho1(){
	
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(ULTRASONIC_ECHOCLK, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_ECHO1; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(ULTRASONIC_ECHOPORT, &GPIO_InitStructure);
		
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void EXTI1_IRQHandler(){
	if (EXTI_GetITStatus(EXTI_Line1) != RESET && transmitFlag1 != 0) {
		if(rising1){
			//RISING
			start1 = micros();
			rising1-=1;
			transmitFlag1+=1;
		}else{
			//FALLING
			delta1 = micros()-start1;
			printf("DT1:%u \n", delta1);
			rising1+=1;
		}
		transmitFlag1--;
		EXTI_ClearITPendingBit(EXTI_Line1); 
	}	
}




void initEcho2(){
	
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(ULTRASONIC_ECHOCLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_ECHO2; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(ULTRASONIC_ECHOPORT, &GPIO_InitStructure);
		
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);
	EXTI_InitStructure.EXTI_Line = EXTI_Line2;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void EXTI2_IRQHandler(){
	if (EXTI_GetITStatus(EXTI_Line2) != RESET && transmitFlag2 != 0) {
		if(rising2){
			//RISING
			start2 = micros();
			rising2-=1;
			transmitFlag2+=1;
		}else{
			//FALLING
			delta2 = micros()-start2;
			printf("DT2:%u \n", delta2);
			rising2+=1;
		}
		transmitFlag2--;
	}	
	EXTI_ClearITPendingBit(EXTI_Line2); 
}

void initVoltageCompare(){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(ULTRASONIC_VCOMPARECLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_VCOMPARE1 | ULTRASONIC_VCOMPARE2; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(ULTRASONIC_VCOMPAREPORT, &GPIO_InitStructure);
	
	DAC_InitTypeDef  DAC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_Init(DAC_Channel_2, &DAC_InitStructure);
	DAC_Cmd(DAC_Channel_2, ENABLE);
	
	DAC_SetChannel1Data(DAC_Align_12b_R, voltageCompare1_Threshold);//max number 4095 ~> Vref i.e 3.3V
	DAC_SetChannel2Data(DAC_Align_12b_R, voltageCompare2_Threshold);
}
