#include "bsp_ultrasonic.h"
#include "pin_configuration.h"
#include "bsp_time.h"
#include <stdio.h>
#include "PGA460_USSC.h"

//STACK DATA STRUCTURE FUNCTIONS
/*------------------------------------------------- Stack Functions -----
 |  Stack_Init 							: Initialize stack
 |  Stack_Push 							: Add temperature, speedOfSound, digitalDelay data to top of stack
 |  Stack_Pop 							: Delete last data of stack
 |  Stack_Temperature_Avg 	: Return avg temperature in stack 
 *-------------------------------------------------------------------*/
void Stack_Init(Stack *S)
{
  S->size = 0;
}

void Stack_Push(Stack *S, double temperature,double speedOfSound,double  digitalDelay, byte address)
{
	if (S->size < STACK_MAX){
		S->temperature[S->size] = temperature;
		S->speedOfSound[S->size] = speedOfSound;
		S->address[S->size] = address;
		S->size++;
	}else{
		printf("Error: stack full \n");
	}
}

void Stack_Pop(Stack *S)
{
    if (S->size == 0)
        printf("Error: stack empty \n");
    else
        S->size--;
}

double Stack_Temperature_Avg (Stack *S){
	double sum = 0;
  for(int i=0; i < S->size; i++){
      sum += S->temperature[i];
  }	
	return (sum/S->size); 
}



//ULTRASONIC FUNCTIONS

/*------------------------------------------------- speedSoundByTemp -----
 | Function speedSoundByTemp
 |
 |  Purpose:  Determine the speed of sound by temperature sensor reading inside PGA460Q1
 |
 |  Parameters:
 |		temp (IN) -- temperature input. Get this value from the system diagnosis of the Die
 |	Return:
 |		speed of sound (meter/second)
 *-------------------------------------------------------------------*/
double speedSoundByTemp(double temp){
	if(temp >= 0){
		return temp*0.6+331;
	}else{
		return temp*(-0.6)+331;
	}
}
/*------------------------------------------------- configPGA460 -----
 |  Function configPGA460
 |
 |  Purpose:  Initialize PGA460Q1. Initializing process can only be done at one chip at a time, can't be in a bus setting
 |
 |  Parameters:
 |		mode (IN) -- sets communication mode. 
 |			0=UART 
 |			1=TCI 
 | 		baud (IN) -- PGA460 accepts a baud rate of 9600 to 115.2k bps
 | 		uartAddrUpdate (IN) -- PGA460 address range from 0 to 7
 |			if configPGA460 param 0, this param has to be filled with the address of ultrasonic to be initialize
 | 		detectAddr (IN) -- retrieve uart addresses that are connected in the bus
 |			0=don't run
 |			1=run
 | 		temperatureSetting (IN) -- setting up temperature variable for speed of sound calculation
 |			0=use Die temperature from diagnosis as temperature variable to determine speed of sound
 |			1=use hardcoded 27'C as temperature variable to determine speed of sound
 | 		runDiag (IN) -- run system diagnostic and print the result
 |			0=No
 |			1=Yes
 | 		runEDD (IN) -- retrieve 128 raw echo data and print the result
 |			0=No
 |			1=Yes
 | 		tempStack (IN) -- temperature reading stack data structure to keep recorded temp reading
 |
 |	Return:
 |			0 = Success Unsuccessfull
 |			1	=	Setup Succesfull
 *-------------------------------------------------------------------*/
bool configPGA460(byte mode, uint32_t baud, byte uartAddrUpdate, int detectAddr, int temperatureSetting, int runDiag, int runEDD, Stack * tempStack){
	double diagnosisResult, temperature;
	//Set communication interface of PGA460 and set the ultrasonic UART addr
	initSTM32F1PGA460(mode,baud,CNFG_PGA460,uartAddrUpdate);
	
	//read available ultrasonic address, setup other params, and burn EEPROM
	bool flag = 0 ;
	byte rd;
	short uartAddrConnected;
	for(short i=0;i<8;i++){
		rd = registerRead(0x1f,i);
		if(rd != 0)
		{
			uartAddrConnected = (rd & 0xe0)>>5;
			if(detectAddr){
				printf("uart address %x connected \r\n",uartAddrConnected);
			}
			//bulk threshold write
			initThresholds(THRESHOLD_CUSTOM,uartAddrConnected);
			//bulk user EEPROM write
			defaultPGA460(TRANSDUCER_CUSTOM,uartAddrConnected);
			//bulk TVG write : 55db - 56 db
			initTVG(AGR_52_84,TVG_CUSTOM_3,uartAddrConnected);
			//with or without run system diagnostic option, need to get temperature to determine speed of sound
			if(temperatureSetting == ROOM_TEMPERATURE){
				temperature = 23;
			}else{
				temperature = runDiagnostics(0,2,uartAddrConnected);
			}			
			double speedOfSound = speedSoundByTemp(temperature);
			double digitalDelay = 0.00005 * speedOfSound;
			Stack_Push(tempStack, temperature, speedOfSound, digitalDelay, uartAddrConnected);
			//run system diagnostic if necessary
			if(runDiag){
				printf("SYSTEM DIAGNOSTIC \r\n");
				diagnosisResult = runDiagnostics(1,0,uartAddrConnected);
				printf("Address %x - Frequency: %f Khz\r\n",uartAddrConnected, diagnosisResult);
				diagnosisResult = runDiagnostics(0,1,uartAddrConnected);
				printf("Address %x - Decay Period: %f us\r\n",uartAddrConnected,diagnosisResult);
				printf("Address %x - Die Temperature : %f C\r\n",uartAddrConnected,temperature);
				diagnosisResult = runDiagnostics(0,3,uartAddrConnected);
				printf("Address %x - Noise Level: %f \r\n",uartAddrConnected,diagnosisResult);
			}
			
			//burn EEPROM
			delay_ms(50);
			byte burnStat = burnEEPROM(uartAddrConnected);
			if(burnStat == true){
				printf("Address %x - EEPROM programmed successfully. \r\n", uartAddrConnected);
			}else{
				printf("Address %x - EEPROM program failed. \r\n", uartAddrConnected);
				return 0;
			}
			
			//run echo data dump if necessary
			if(runEDD){
				printf("Address %x - EED: ",uartAddrConnected);
				runEchoDataDump(LONG_DIST_MEASUREMENT, uartAddrConnected); //run prset 1 
				for (int n = 0; n < 128; n++){
					byte echoDataDumpElement = pullEchoDataDump(n,uartAddrConnected);
					printf("%d ",echoDataDumpElement);
				}
				printf("\r\n");
			}
			
			//minimum 1 uart address detected
			flag = 1;
		}
		
		//if none UART addres connected/read
		else if(i == 7 && flag != 1)
		{
			if(detectAddr){
				printf("ERROR - can't read uart address!\r\n");
			}
			return 0;
		}
	}	
	
	//setup success
	return 1;
}

/*------------------------------------------------- initPGA460 -----
 |  Function initPGA460
 |
 |  Purpose:  Initialize PGA460Q1 for direct run (you should have configured the uart adrress of each module connected in the bus) 
 |
 |  Parameters:
 |		mode (IN) -- sets communication mode. 
 |			0=UART 
 |			1=TCI 
 | 		baud (IN) -- PGA460 accepts a baud rate of 9600 to 115.2k bps 
 | 		detectAddr (IN) -- retrieve uart addresses that are connected in the bus
 |			0=don't run
 |			1=run
 | 		temperatureSetting (IN) -- setting up temperature variable for speed of sound calculation
 |			0=use Die temperature from diagnosis as temperature variable to determine speed of sound
 |			1=use hardcoded 27'C as temperature variable to determine speed of sound
 | 		runDiag (IN) -- run system diagnostic and print the result
 |			0=No
 |			1=Yes
 | 		runEDD (IN) -- retrieve 128 raw echo data and print the result
 |			0=No
 |			1=Yes
 | 		tempStack (IN) -- temperature reading stack data structure to keep recorded temp reading
 |
 |	Return:
 |			0 = Success Unsuccessfull
 |			1	=	Setup Succesfull
 *-------------------------------------------------------------------*/
bool initPGA460(byte mode, uint32_t baud, int temperatureSetting, int detectAddr, int runDiag, int runEDD, Stack * tempStack){

	double diagnosisResult, temperature;	
	//Set communication interface of PGA460
	initSTM32F1PGA460(mode,baud,RUN_PGA460,0);
	
	//read available ultrasonic address, setup other params, and burn EEPROM
	bool flag = 0 ;
	byte rd;
	short uartAddrConnected;
	for(short i=0;i<8;i++){
		rd = registerRead(0x1f,i);
		if(rd != 0)
		{
			uartAddrConnected = (rd & 0xe0)>>5;
			if(detectAddr){
				printf("uart address %x connected \r\n",uartAddrConnected);
			}
			//bulk threshold write
			initThresholds(THRESHOLD_CUSTOM,uartAddrConnected);
			//bulk user EEPROM write
			defaultPGA460(TRANSDUCER_CUSTOM,uartAddrConnected);
			//bulk TVG write
			initTVG(AGR_52_84,TVG_50,uartAddrConnected);
			//with/without run system diagnostic option, need to get temperature to determine speed of sound
			if(temperatureSetting == ROOM_TEMPERATURE){
				temperature = 27;
			}else{
				temperature = runDiagnostics(0,2,uartAddrConnected);
			}		
			double speedOfSound = speedSoundByTemp(temperature);
			double digitalDelay = 0.00005 * speedOfSound;
			Stack_Push(tempStack, temperature, speedOfSound, digitalDelay, uartAddrConnected);
			//run system diagnostic if necessary
			if(runDiag){
				printf("SYSTEM DIAGNOSTIC \r\n");
				diagnosisResult = runDiagnostics(1,0,uartAddrConnected);
				printf("Address %x - Frequency: %f Khz\r\n",uartAddrConnected, diagnosisResult);
				diagnosisResult = runDiagnostics(0,1,uartAddrConnected);
				printf("Address %x - Decay Period: %f us\r\n",uartAddrConnected,diagnosisResult);
				printf("Address %x - Die Temperature : %f C\r\n",uartAddrConnected,temperature);
				diagnosisResult = runDiagnostics(0,3,uartAddrConnected);
				printf("Address %x - Noise Level: %f \r\n",uartAddrConnected,diagnosisResult);
			}
			
			//burn EEPROM
			delay_ms(50);
			byte burnStat = burnEEPROM(uartAddrConnected);
			if(burnStat == true){
				printf("Address %x - EEPROM programmed successfully. \r\n", uartAddrConnected);
			}else{
				printf("Address %x - EEPROM program failed. \r\n", uartAddrConnected);
				return 0;
			}
			
			//run echo data dump if necessary
			if(runEDD){
				printf("Address %x - EED: ",uartAddrConnected);
				runEchoDataDump(1, uartAddrConnected); //run prset 1 
				for (int n = 0; n < 128; n++){
					byte echoDataDumpElement = pullEchoDataDump(n,uartAddrConnected);
					printf("%d, ",echoDataDumpElement);
				}
				printf("\r\n");
			}
			
			//minimum 1 uart address detected
			flag = 1;
		}
		
		//if none UART addres connected/read
		else if(i == 7 && flag != 1)
		{
			if(detectAddr){
				printf("ERROR - can't read uart address!\r\n");
			}
			return 0;
		}
	}	
	
	//setup success
	return 1;
	
}

void autoThresholdRun(byte mode, byte uartIndex, byte noiseMargin, byte thrTimeIndex, byte thrPoints, int copyThr){
	delay_ms(500);
	printf("Automatically configuring the preset's threshold map... \r\n");
	autoThreshold(mode,noiseMargin*8,thrTimeIndex,thrPoints,1,uartIndex);

	if(copyThr == 0 || copyThr == 2){
		printf("Copying threshold map to EEPROM memory space... \r\n");
		eepromThreshold(mode+1,false, uartIndex);
	}
	
	byte thrAddr = 0x5F; // beginning of threshold memory space
  if (mode == 1 || mode == 3){
		thrAddr += 16;
	}
	printf("Autoset Threshold Results (HEX): ");
  for (int i = 0; i <16; i++){
		printf("%x", registerRead(i + thrAddr,uartIndex));
    printf(" ");		
		if(i == 5 || i == 10 || i == 14){
			printf(" -- ");
		}
	}
	printf("\r\n Autoset Threshold Finish \r\n");
}


