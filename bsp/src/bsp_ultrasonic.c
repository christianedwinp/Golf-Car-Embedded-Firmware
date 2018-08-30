#include "bsp_ultrasonic.h"
#include "pin_configuration.h"
#include "bsp_time.h"
#include <stdio.h>
#include "PGA460_USSC.h"
#include "stack.h"

/*------------------------------------------------- InitPGA460 -----
 |  Function InitPGA460
 |
 |  Purpose:  Initialize PGA460Q1, either single program or multiple program
 |
 |  Parameters:
 |		configPGA460 (IN) -- set internal parameter and communication interface of a single PGA460 chip
 |			0=No, means you have set the internal parameter of the PGA460 chip and will drive the board directly
 |			1=Yes, means you will set the internal parameter of the PGA460 chip
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
 *-------------------------------------------------------------------*/
void InitPGA460(int configPGA460, byte uartAddrUpdate, int detectAddr, int runDiag, int runEDD){
	double result;

	//Set communication interface of PGA460 : OWU mode
	initSTM32F1PGA460(2,19200);

	if(configPGA460){
		UltraDevNum = 0;
		//set address to the ultrasonic
		SetPGAAddress(uartAddrUpdate);
	}
	
	//read available ultrasonic address
	if(detectAddr){
		short i ;
		byte rd;
		for(i=0;i<8;i++)
		{
			rd = registerRead(0x1f,i);
			if(rd != 0)
			{
				PGAUartAddr = rd & 0xe0;
				printf("uart address %x connected \n",PGAUartAddr>>5);
			}
			else if(i == 7)
			{
				printf("can't read uart address!\n");
			}
		}
	}
		
	//bulk threshold write : threshold 50%
	initThresholds(1,uartAddrUpdate);
	//bulk user EEPROM write : set using custom transducer
	defaultPGA460(2,uartAddrUpdate);
	//bulk TVG write
	initTVG(2,1,uartAddrUpdate);
	//with/without run system diagnostic option, need to get temperature to determine speed of sound
	double temperature = runDiagnostics(0,2,uartAddrUpdate);
	Stack_Push(temperatureReading, temperature);

	//run system diagnostic if necessary
	if(runDiag){
		printf("SYSTEM DIAGNOSTIC \n");
		result = runDiagnostics(1,0,uartAddrUpdate);
		printf("Address %hhx - Frequency: %f Khz\n",uartAddrUpdate, result);
		result = runDiagnostics(0,1,uartAddrUpdate);
		printf("Address %hhx - Decay Period: %f us\n",uartAddrUpdate,result);
		printf("Address %hhx - Die Temperature : %f C\n",uartAddrUpdate,temperature);
		result = runDiagnostics(0,3,uartAddrUpdate);
		printf("Address %hhx - Noise Level: %f \n",uartAddrUpdate,result);
	}

	//burn EEPROM
	delay_ms(50);
	byte burnStat = burnEEPROM(uartAddrUpdate);
	if(burnStat == true){
		printf("Address %hhx - EEPROM programmed successfully.", uartAddrUpdate,);
	}else{
		printf("Address %hhx - EEPROM program failed.", uartAddrUpdate,);
	}

	//run echo data dump if necessary
	if(runEDD){
		printf("Address %hhx - EED: ",uartAddrUpdate);
		runEchoDataDump(1, uartAddrUpdate); //run prset 1 
		for (int n = 0; n < 128; n++){
			byte echoDataDumpElement = pullEchoDataDump(n, );
			printf("%d, ",echoDataDumpElement);
		}
		printf("\n");
	}
}
