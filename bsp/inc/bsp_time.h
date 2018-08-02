#ifndef _TIME_H_
#define _TIME_H_

#include "stm32f10x.h"

uint32_t millis(void);
uint32_t micros(void);

uint32_t time_nowMs(void);
uint64_t time_nowUs(void);

void delay_ms (u32);
void delay_us (u32);

void Systick_Init(void);

#endif 
