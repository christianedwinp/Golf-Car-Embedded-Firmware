#include "bsp_ultrasonic.h"
#include "pin_configuration.h"
#include <stdio.h>

volatile uint16_t ADC_values[4];

int ultrasonicProgState = 0;
int muxNumber = 1;
int muxChannel = 1;
int ultrasonicNumber = 0;
float distance[8] = {0,0,0,0,0,0,0,0};
uint32_t transmitTime, receiveTime, previousReceiveTime;

void selectMuxChannel(int channel){
	switch(channel){
		case 1:
			//1: 00
		MULT_CHANNEL_SELECT_PORT->BRR = MULT_CHANNEL_SELECT_A;
		MULT_CHANNEL_SELECT_PORT->BRR = MULT_CHANNEL_SELECT_B;			
		break;
		case 2:
			//2: 01
		MULT_CHANNEL_SELECT_PORT->BRR 	= MULT_CHANNEL_SELECT_A;
		MULT_CHANNEL_SELECT_PORT->BSRR = MULT_CHANNEL_SELECT_B;
		break;
		case 3:
			//3: 10
		MULT_CHANNEL_SELECT_PORT->BSRR = MULT_CHANNEL_SELECT_A;
		MULT_CHANNEL_SELECT_PORT->BRR 	= MULT_CHANNEL_SELECT_B;
		break;
		case 4:
			//4: 11
		MULT_CHANNEL_SELECT_PORT->BSRR = MULT_CHANNEL_SELECT_A;
		MULT_CHANNEL_SELECT_PORT->BSRR	= MULT_CHANNEL_SELECT_B;
		break;
		default:
		printf("Channel number outbound, only input 1/2/3/4");
	}
}

void selectMux(int mult){
	switch(mult){
		case 1:
		MULTPORT->BRR  = MULT2;
		MULTPORT->BSRR = MULT1;			
		break;
		case 2:
		MULTPORT->BRR  = MULT1;
		MULTPORT->BSRR = MULT2;
		break;
		default:
		printf("Multiplexer number outbound, only input 1/2");
	}
}

void muxPinInit(){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(MULTCLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = MULT1 | MULT2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MULTPORT, &GPIO_InitStructure);
}

void muxChannelPinInit(){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(MULT_CHANNEL_SELECT_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = MULT_CHANNEL_SELECT_A | MULT_CHANNEL_SELECT_B;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MULT_CHANNEL_SELECT_PORT, &GPIO_InitStructure);
}

void ultrasonicTransmitPinInit(){
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure_TimerInterrupt;

	//Master Timer GPIO Setting
	RCC_APB2PeriphClockCmd(ULTRASONIC_MASTER_OUTPUT_CLK | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_MASTER_OUTPUT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	//Master Timer timebase setting
	RCC_APB1PeriphClockCmd(ULTRASONIC_MASTER_TIMER_CLK, ENABLE);
	TIM_TimeBaseStructure.TIM_Prescaler = 9999; // clk = 72 MHz / (9999 + 1) = 7.2 khz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_Period = 256; // freq = 7.2 khz / (256 + 1) = 28.02 hz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(ULTRASONIC_MASTER_TIMER, &TIM_TimeBaseStructure);

	//Setting Master Timer Output Compare
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 128; // 257 * 50% = 128
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//Interrupt for every 28.02 Hz to change multiplexer or multiplexer channels
    NVIC_InitStructure_TimerInterrupt.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure_TimerInterrupt.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure_TimerInterrupt.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure_TimerInterrupt.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure_TimerInterrupt);

    TIM_Cmd(ULTRASONIC_MASTER_TIMER, ENABLE);
	//Enable Master Slave mode for Master Timer
	TIM_SelectMasterSlaveMode(ULTRASONIC_MASTER_TIMER, TIM_MasterSlaveMode_Enable);
	//Select Output Trigger for Master Timer
	TIM_SelectOutputTrigger(ULTRASONIC_MASTER_TIMER, TIM_TRGOSource_Update);

	//Slave Timer GPIO Setting
	RCC_APB2PeriphClockCmd(ULTRASONIC_SLAVE_CLK | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_SLAVE_OUTPUT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	//Slave Timer timebase setting
	RCC_APB1PeriphClockCmd(ULTRASONIC_SLAVE_TIMER_CLK, ENABLE);
	TIM_TimeBaseStructure.TIM_Prescaler = 1; // clk = 72M/(1 + 1) = 36Mhz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_Period = 749; // freq = 36Mhz / 749+1 = 48 Khz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 7; // pulse repeated = #numberOfWantedRepeatedPulseWanted - 1
	TIM_TimeBaseInit(ULTRASONIC_SLAVE_TIMER, &TIM_TimeBaseStructure);
		
	//Slave timer output compare setting
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 375; // 750 * 50% = 375
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//Select Slave Mode for slave timer
	TIM_SelectOnePulseMode(ULTRASONIC_SLAVE_TIMER, TIM_OPMode_Single);
	TIM_SelectSlaveMode(ULTRASONIC_SLAVE_TIMER, TIM_SlaveMode_Trigger);
	//Select Input Trigger for slave timer
	TIM_SelectInputTrigger(ULTRASONIC_SLAVE_TIMER, ULTRASONIC_MASTERSLAVE_ITR);

	GPIO_Init(ULTRASONIC_SLAVE_PORT, &GPIO_InitStructure);
	TIM_OC1Init(ULTRASONIC_SLAVE_TIMER, &TIM_OCInitStructure);
	TIM_Cmd(ULTRASONIC_SLAVE_TIMER, ENABLE);
}

//to handle interrupt every 28.02 Hz
void TIM2_IRQHandler()
{
    if (TIM_GetITStatus(ULTRASONIC_MASTER_TIMER, TIM_IT_Update) != RESET)
    {
        //clear interrupt bit
        TIM_ClearITPendingBit(ULTRASONIC_MASTER_TIMER, TIM_IT_Update);
        
        //INTERRUPT ROUTINE
        if(previousReceiveTime == receiveTime){
        	//if previous receive time doesnt change, that means no echo detected, so distance unlimited/max
        	distance[ultrasonicNumber] = 100;
        }else{
        	//if previous receive time is different, then calc normaly distance
        	distance[ultrasonicNumber] = ultrasonicComputeDistance(transmitTime, receiveTime);
        }
        //update counter variable
        muxChannel++;
		ultrasonicNumber++;
		if(muxChannel % 5 == 0){
			if(muxNumber == 0){
				muxNumber++;
			}else{
				muxNumber--;
			}
			muxChannel = 1;
		}
		if(ultrasonicNumber == 9){
			ultrasonicNumber = 1;
		}

		ultrasonicProgState = 0;
		previousReceiveTime = receiveTime;

		//select multiplexer based on the updated counter variable
		selectMux(muxNumber);
		//select multiplexer channel based on the updated counter variable
        selectMuxChannel(muxChannel);
    }
}

void ultrasonicReceiverPinInit(){
	GPIO_InitTypeDef GPIO_InitStructure;	
	ADC_InitTypeDef ADC_InitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure_ADCInterrupt;	
	DMA_InitTypeDef  DMA_InitStructure;

	//ADC pin config
	RCC_APB2PeriphClockCmd(ULTRASONIC_INPUT_CLK | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ULTRASONIC_INPUT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ULTRASONIC_INPUT_PORT, &GPIO_InitStructure);

	//configure interrupt
	// NVIC_InitStructure_ADCInterrupt.NVIC_IRQChannel = ADC1_2_IRQn;
	// NVIC_InitStructure_ADCInterrupt.NVIC_IRQChannelPreemptionPriority = 0;
	// NVIC_InitStructure_ADCInterrupt.NVIC_IRQChannelSubPriority = 0;
	// NVIC_InitStructure_ADCInterrupt.NVIC_IRQChannelCmd = ENABLE;
	// NVIC_Init(&NVIC_InitStructure_ADCInterrupt);
	
	//ADC config
	RCC_APB2PeriphClockCmd(ULTRASONIC_INPUT_ADC_CLK, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //APB2 clock use PCLK2(72 MHz), ADCCLK = PCLK2/6 = 72/6 = 12 MHz
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ULTRASONIC_INPUT_ADC_PORT, &ADC_InitStructure);
	ADC_RegularChannelConfig(ULTRASONIC_INPUT_ADC_PORT, ULTRASONIC_INPUT_ADC_CHANNEL, 1, ADC_SampleTime_55Cycles5);
	
	//set ADC Watchdog 
	// NOTE : MAY NOT NEED WD
	// ADC_AnalogWatchdogThresholdsConfig(ULTRASONIC_INPUT_ADC_PORT, ADC_WD_HIGHTHRESHOLD, ADC_WD_LOWTHRESHOLD);
	// ADC_AnalogWatchdogSingleChannelConfig(ULTRASONIC_INPUT_ADC_PORT, ULTRASONIC_INPUT_ADC_CHANNEL);
	// ADC_AnalogWatchdogCmd(ULTRASONIC_INPUT_ADC_PORT, ENABLE);
	// ADC_ITConfig(ULTRASONIC_INPUT_ADC_PORT, ADC_IT_AWD, ENABLE);
	
	ADC_Cmd(ULTRASONIC_INPUT_ADC_PORT, ENABLE);

	//ADC Calibration
	ADC_ResetCalibration(ULTRASONIC_INPUT_ADC_PORT);
	while(ADC_GetResetCalibrationStatus(ULTRASONIC_INPUT_ADC_PORT));
	ADC_StartCalibration(ULTRASONIC_INPUT_ADC_PORT);
	while(ADC_GetCalibrationStatus(ULTRASONIC_INPUT_ADC_PORT));
	
	//DMA config
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel1); //reset DMA1 channe1 to default values
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //channel will be used for memory to memory transfer
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //setting normal mode (non circular)
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //medium priority
 	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //source and destination data size word=32bit
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //source and destination data size word=32bit
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //automatic memory destination increment enable
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //source address increment disable
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //Location assigned to peripheral register will be source
  DMA_InitStructure.DMA_BufferSize = 4; //chunk of data to be transfered 4 hex: ADC conversion return 16 bits value
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC1->DR));//source and destination start addresses
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_values;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure); //send values to DMA registers
  // DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); // Enable DMA1 Channel Transfer Complete interrupt
  DMA_Cmd(DMA1_Channel1, ENABLE); //Enable the DMA1 - Channel1

  //Enable DMA1 channel IRQ Channel
  // NVIC_InitStructure_ADCInterrupt.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  // NVIC_InitStructure_ADCInterrupt.NVIC_IRQChannelPreemptionPriority = 0;
  // NVIC_InitStructure_ADCInterrupt.NVIC_IRQChannelSubPriority = 0;
  // NVIC_InitStructure_ADCInterrupt.NVIC_IRQChannelCmd = ENABLE;
  // NVIC_Init(&NVIC_InitStructure_ADCInterrupt);

	ADC_SoftwareStartConvCmd(ULTRASONIC_INPUT_ADC_PORT, ENABLE);
	while(ADC_GetSoftwareStartConvStatus(ULTRASONIC_INPUT_ADC_PORT));
}

// void ADC1_2_IRQHandler(){
// 	ADC_ClearITPendingBit(ULTRASONIC_INPUT_ADC_PORT, ADC_IT_AWD);
// }

void BSP_UltrasonicInit(){
	//MULTIPLEXER PIN INITIALIZATION
	muxPinInit();
	
	//MULTIPLEXER CHANNEL PIN INITIALIZATION
	muxChannelPinInit();

	/* ULTRASONIC TRANSMIT PIN INITIALIZATION
	 * Generate 8 pulse of 48 kHz 50% duty cycle for every 35 ms
	 * TIM2 -> TIM3
	 * Slave Timer  : 8 pulse PWM 48 kHz 50%
	 * Master Timer : PWM period 35 ms
	 * Master timer has interrupt every period to change multiplexer or multiplexer channel */
	ultrasonicTransmitPinInit();

	//ULTRASONIC RECEIVER PIN INITIALIZATION
	ultrasonicReceiverPinInit();
}

float ultrasonicComputeDistance(uint32_t transmitTime, uint32_t receiveTime){
	return 0.34 * (receiveTime - transmitTime);
}
