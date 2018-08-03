#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include "bsp.h" 


extern volatile uint8_t gTimerFlag;

void BSP_TimerInit(uint16_t freq);
void BSP_Timer8PWM(void);
#endif
