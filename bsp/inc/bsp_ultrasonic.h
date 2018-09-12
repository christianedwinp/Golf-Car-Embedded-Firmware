#ifndef __BSP_ULTRASONIC_H
#define __BSP_ULTRASONIC_H

#include "bsp.h" 

#define byte unsigned char
#define STACK_MAX 8
#define MAX_OBJECT_DETECTED 8
#define CNFG_PGA460 	1
#define RUN_PGA460		0
#define CHECK_ADDRESS_ON		1
#define CHECK_ADDRESS_OFF		0
#define SYS_DIAGNOSIS_ON 		1
#define SYS_DIAGNOSIS_OFF 	0
#define ECHO_DATA_DUMP_ON 	1
#define ECHO_DATA_DUMP_OFF	0
#define SHORT_DIST_MEASUREMENT 0
#define LONG_DIST_MEASUREMENT 1

//STACK DATA STRUCTURE FUNCTIONS
struct Stack {
    byte 		address[STACK_MAX];
		double  temperature[STACK_MAX];
	  double  speedOfSound[STACK_MAX];
		double  digitalDelay[STACK_MAX];
		double  distance[STACK_MAX][MAX_OBJECT_DETECTED];
		double  width[STACK_MAX][MAX_OBJECT_DETECTED];
		double  peak[STACK_MAX][MAX_OBJECT_DETECTED];
	  byte 		objectIsDetected[STACK_MAX];
    int     size;
};
typedef struct Stack Stack;
void Stack_Init(Stack *S);
void Stack_Push(Stack *S, double temperature,double speedOfSound, double  digitalDelay, byte address);
void Stack_Pop(Stack *S);
double Stack_Avg(Stack *S);

//ULTRASONIC SPECIFIC FUNCTIONS
_Bool configPGA460(byte mode, uint32_t baud, byte uartAddrUpdate, int detectAddr, int runDiag, int runEDD, Stack * tempStack);
_Bool initPGA460(byte mode, uint32_t baud, int detectAddr, int runDiag, int runEDD, Stack * tempStack);
double speedSoundByTemp(double temp);

#endif
