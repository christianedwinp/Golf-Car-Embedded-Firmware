#ifndef __BSP_ULTRASONIC_H
#define __BSP_ULTRASONIC_H

#include "bsp.h" 
#include "stack.h"
#include <stdio.h>

#define byte unsigned char
#define CNFG_PGA460 	1
#define RUN_PGA460		0


_Bool configPGA460(byte mode, uint32_t baud, byte uartAddrUpdate, int detectAddr, int runDiag, int runEDD, Stack * tempStack);

_Bool initPGA460(byte mode, uint32_t baud, int detectAddr, int runDiag, int runEDD, Stack * tempStack);

#endif
