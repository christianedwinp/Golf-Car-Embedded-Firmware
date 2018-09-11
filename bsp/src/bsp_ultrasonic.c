#include "bsp_ultrasonic.h"
#include "pin_configuration.h"
#include "bsp_time.h"
#include <stdio.h>
#include "PGA460_USSC.h"
#include "stack.h"

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
bool configPGA460(byte mode, uint32_t baud, byte uartAddrUpdate, int detectAddr, int runDiag, int runEDD, Stack * tempStack){
	double diagnosisResult;
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
			//bulk threshold write : threshold 50%
			initThresholds(1,uartAddrConnected);
			//bulk user EEPROM write : set using custom transducer
			defaultPGA460(2,uartAddrConnected);
			//bulk TVG write
			initTVG(2,1,uartAddrConnected);
			//with/without run system diagnostic option, need to get temperature to determine speed of sound
			double temperature = runDiagnostics(0,2,uartAddrConnected);
			Stack_Push(tempStack, temperature, uartAddrConnected);
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
bool initPGA460(byte mode, uint32_t baud, int detectAddr, int runDiag, int runEDD, Stack * tempStack){
	double diagnosisResult;	
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
			//bulk threshold write : threshold 50%
			initThresholds(1,uartAddrConnected);
			//bulk user EEPROM write : set using custom transducer
			defaultPGA460(2,uartAddrConnected);
			//bulk TVG write
			initTVG(2,1,uartAddrConnected);
			//with/without run system diagnostic option, need to get temperature to determine speed of sound
			double temperature = runDiagnostics(0,2,uartAddrConnected);
			Stack_Push(tempStack, temperature,uartAddrConnected);
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
