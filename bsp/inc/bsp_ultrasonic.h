#ifndef __BSP_ULTRASONIC_H
#define __BSP_ULTRASONIC_H

#include "bsp.h" 

#define  byte unsigned char

void InitPGA460(int configPGA460, byte uartAddrUpdate, int detectAddr, int runDiag, int runEDD);


#endif
