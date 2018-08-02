#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "bsp.h" 

extern int16_t gEncoder[4];

void BSP_EncoderInit(void);
void BSP_EncoderRead(void);

#endif
