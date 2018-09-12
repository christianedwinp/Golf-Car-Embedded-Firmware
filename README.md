# Golf Car Ultrasonic & Encoder Project

## Overview
- Hardware: STM32F103VET6
- IDE: ARM-MDK
- Library: STM32F4xx_DSP_StdPeriph_Lib_V1.8.0  

## Pin&Function Map
|SYSTEM			| GPIO PORT | GPIO PIN	| FUNCTION USE
|---			|----		| ----		| ----		
|MCO_DEBUG		|	GPIOA	|	8		|	MCO
|USART_RX		|	GPIOA	|	10		|	USART1_RX
|USART_TX		|	GPIOA	|	9		|	USART1_TX
|LED 1			|	GPIOB	|	13		|	OUTPUT GPIO
|LED 2			|	GPIOB	|	14		|	OUTPUT GPIO
|ULTRASONIC_RX	|	GPIOA	|	2		|	USART2_RX
|ULTRASONIC_TX	|	GPIOA	|	3		|	USART2_TX

## HOW TO USE
1. connect Ultrasonic Module TX to PA3,RX to PA2
2. To read UART message, connect USB-TTL TX to PA10 & USB-TTL RX to PA9 