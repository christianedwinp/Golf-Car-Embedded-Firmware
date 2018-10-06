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
#define ROOM_TEMPERATURE 1
#define DIAG_TEMPERATURE 0
#define THRESHOLD_25 0
#define THRESHOLD_50 1
#define THRESHOLD_75 2
#define THRESHOLD_CUSTOM 3
#define TVG_25 0
#define TVG_50 1
#define TVG_75 2
#define TVG_CUSTOM_1 3
#define TVG_CUSTOM_2 4
#define TVG_CUSTOM_3 5
#define TRANSDUCER_MA58MF147N 0
#define TRANSDUCER_MA40H1SR 1
#define TRANSDUCER_CUSTOM 2
#define AGR_58_90 3
#define AGR_52_84 2
#define AGR_46_78 1
#define AGR_32_64 0

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
_Bool configPGA460(byte mode, uint32_t baud, byte uartAddrUpdate, int detectAddr, int temperatureSetting, int runDiag, int runEDD, Stack * tempStack);
_Bool initPGA460(byte mode, uint32_t baud, int detectAddr, int temperatureSetting, int runDiag, int runEDD, Stack * tempStack);
void autoThresholdRun(byte mode, byte uartIndex, byte noiseMargin, byte thrTimeIndex, byte thrPoints,int copyThr);
double speedSoundByTemp(double temp);

#endif
