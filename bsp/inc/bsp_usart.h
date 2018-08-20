#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "stdio.h"	
#include "bsp.h" 
#define UsartRxBuffSize 50
void BSP_UsartInit(u32 bound);
void BSP_Usart2Init(int baud);
short Usart2_DataAvailable(void);
void Usart_Flush(void);
int read(void);
void Usart2Send(unsigned char *Str, int len);
#endif
