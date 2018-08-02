#ifndef __BSP_ULTRASONIC_H
#define __BSP_ULTRASONIC_H

#include "bsp.h" 

#define ADC_WD_HIGHTHRESHOLD		3000
#define ADC_WD_LOWTHRESHOLD			1000

#define ULTRASONIC_THRESHOLD		1000

// #define ADC1_DR    ((uint32_t)0x4001244C)


void selectMuxChannel(int channel);
void selectMux(int mult);
void muxPinInit(void);
void muxChannelPinInit(void);
void ultrasonicTransmitPinInit(void);
void ultrasonicReceiverPinInit(void);
void BSP_UltrasonicInit(void);
void BPS_UltrasonicTransmission(void);
float ultrasonicComputeDistance(uint32_t transmitTime, uint32_t receiveTime);


#endif
