#ifndef __BSP_ULTRASONIC_H
#define __BSP_ULTRASONIC_H

#include "bsp.h" 

//DAC quantization 12 bits ~> 0-4095
//0V corresponds 0, 3.3V corresponds Vref pin supply voltage (in our case 3.3V)
#define voltageCompare1_Threshold	620		
#define voltageCompare2_Threshold	620		

void InitPGA460(void);
void BSP_UltrasonicInit(void);
void initTrig(void);
void initEcho1(void);
void initEcho2(void);
void initVoltageCompare(void);
void initMux(void);
void selectChannel_Mux2(int channel);
void selectChannel_Mux1(int channel);
void switchOffMux(int mux);
void switchOnMux(int mux);


#endif
