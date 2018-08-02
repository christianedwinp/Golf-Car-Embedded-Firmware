#ifndef __Pin_Configuration
 #define __Pin_Configuration

#include "stm32f10x.h" 

/* For device classification, check stm32f10x_it.h file
 * STM32F103VET6 dev board count as STM32F10X_HD (256-514 kb flash)
 * STM32F103RBT6 count as STM32F10X_MD (64-128 kb flash)
 */

//TEST LED
#ifdef STM32F10X_HD 
 #define TEST_LED_1						GPIO_Pin_13
 #define TEST_LED_2						GPIO_Pin_14
 #define TEST_LED_PORT					GPIOB
 #define TEST_LED_CLK					RCC_APB2Periph_GPIOB
#else
 // #define TEST_LED_1						GPIO_Pin_8
 // #define TEST_LED_2						GPIO_Pin_9
 // #define TEST_LED_PORT					GPIOC
 // #define TEST_LED_CLK					RCC_APB2Periph_GPIOC
#endif

// CAN PINS
#ifdef STM32F10X_HD 
 #define CAN_TX							GPIO_Pin_9
 #define CAN_RX							GPIO_Pin_8
 #define CAN_PORT 						GPIOB
 #define CAN_CLK				 		RCC_APB2Periph_GPIOB
#else
 #define CAN_TX							GPIO_Pin_12
 #define CAN_RX							GPIO_Pin_11
 #define CAN_PORT 						GPIOA
 #define CAN_CLK				 		RCC_APB2Periph_GPIOA
#endif

//ENCODER TIMER GENERATOR
#ifdef STM32F10X_HD
 #define ENC_TIMER 						TIM2
 #define ENC_TIMER_CLK					RCC_APB1Periph_TIM2												
#else
 // #define ENC_TIMER 						TIM2
 // #define ENC_TIMER_CLK					RCC_APB1Periph_TIM2		 
#endif

// ENCODER PINS
#ifdef STM32F10X_HD 
 #define BACK_LEFT_CH1					GPIO_Pin_6
 #define BACK_LEFT_CH2					GPIO_Pin_7
 #define BACK_LEFT_PORT					GPIOA
 #define BACK_LEFT_CLK					RCC_APB2Periph_GPIOA
 #define BACK_LEFT_TIMER				TIM3
 #define BACK_LEFT_TIMER_CLK			RCC_APB1Periph_TIM3

 #define BACK_RIGHT_CH1					GPIO_Pin_6
 #define BACK_RIGHT_CH2					GPIO_Pin_7
 #define BACK_RIGHT_PORT				GPIOB
 #define BACK_RIGHT_CLK					RCC_APB2Periph_GPIOB
 #define BACK_RIGHT_TIMER				TIM4
 #define BACK_RIGHT_TIMER_CLK			RCC_APB1Periph_TIM4

 // #define FRONT_LEFT_CH1					GPIO_Pin_6
 // #define FRONT_LEFT_CH2					GPIO_Pin_7
 // #define FRONT_LEFT_PORT				GPIOA
 // #define FRONT_LEFT_CLK					RCC_APB2Periph_GPIOA
 // #define FRONT_LEFT_TIMER				TIM3
 // #define FRONT_LEFT_TIMER_CLK			RCC_APB1Periph_TIM3

 // #define FRONT_RIGHT_CH1				GPIO_Pin_6
 // #define FRONT_RIGHT_CH2				GPIO_Pin_7
 // #define FRONT_RIGHT_PORT				GPIOB
 // #define FRONT_RIGHT_CLK				RCC_APB2Periph_GPIOB
 // #define FRONT_RIGHT_TIMER				TIM4
 // #define FRONT_RIGHT_TIMER_CLK			RCC_APB1Periph_TIM4
#else
 // #define BACK_LEFT_CH1					GPIO_Pin_6
 // #define BACK_LEFT_CH2					GPIO_Pin_7
 // #define BACK_LEFT_PORT					GPIOA
 // #define BACK_LEFT_CLK					RCC_APB2Periph_GPIOA
 // #define BACK_LEFT_TIMER				TIM3
 // #define BACK_LEFT_TIMER_CLK			RCC_APB1Periph_TIM3

 // #define BACK_RIGHT_CH1					GPIO_Pin_6
 // #define BACK_RIGHT_CH2					GPIO_Pin_7
 // #define BACK_RIGHT_PORT				GPIOB
 // #define BACK_RIGHT_CLK					RCC_APB2Periph_GPIOB
 // #define BACK_RIGHT_TIMER				TIM4
 // #define BACK_RIGHT_TIMER_CLK			RCC_APB1Periph_TIM4

 // #define FRONT_LEFT_CH1					GPIO_Pin_6
 // #define FRONT_LEFT_CH2					GPIO_Pin_7
 // #define FRONT_LEFT_PORT				GPIOA
 // #define FRONT_LEFT_CLK					RCC_APB2Periph_GPIOA
 // #define FRONT_LEFT_TIMER				TIM3
 // #define FRONT_LEFT_TIMER_CLK			RCC_APB1Periph_TIM3

 // #define FRONT_RIGHT_CH1				GPIO_Pin_6
 // #define FRONT_RIGHT_CH2				GPIO_Pin_7
 // #define FRONT_RIGHT_PORT				GPIOB
 // #define FRONT_RIGHT_CLK				RCC_APB2Periph_GPIOB
 // #define FRONT_RIGHT_TIMER				TIM4
 // #define FRONT_RIGHT_TIMER_CLK			RCC_APB1Periph_TIM4
#endif

// RS485 PINS
#ifdef STM32F10X_HD 
 #define RS485_TX						GPIO_Pin_10
 #define RS485_RX						GPIO_Pin_11
 #define RS485_TX_RX_PORT				GPIOB
 #define RS485_TX_RX_CLK		 		RCC_APB2Periph_GPIOB
 #define RS485_RE						GPIO_Pin_5
 #define RS485_DE						GPIO_Pin_4
 #define RS485_RE_DE_PORT 				GPIOC
 #define RS485_RE_DE_CLK		 		RCC_APB2Periph_GPIOC
 #define RS485_USART					USART3
 #define RS485_USART_CLK 				RCC_APB1Periph_USART3
#else
 #define RS485_TX						GPIO_Pin_10
 #define RS485_RX						GPIO_Pin_11
 #define RS485_TX_RX_PORT				GPIOB
 #define RS485_TX_RX_CLK		 		RCC_APB2Periph_GPIOB
 #define RS485_RE_DE					GPIO_Pin_12
 // #define RS485_RE						GPIO_Pin_5
 // #define RS485_DE						GPIO_Pin_4
 #define RS485_RE_DE_PORT 				GPIOB
 #define RS485_RE_DE_CLK		 		RCC_APB2Periph_GPIOB
 #define RS485_USART					USART3
 #define RS485_USART_CLK 				RCC_APB1Periph_USART3
#endif

//USART PINS
#ifdef STM32F10X_HD 
 #define USART_TX						GPIO_Pin_9
 #define USART_RX						GPIO_Pin_10
 #define USART_PORT 					GPIOA
 #define USART_IO_CLK			 		RCC_APB2Periph_GPIOA
 #define USART_CHANNEL					USART1
 #define USART_CLK 						RCC_APB2Periph_USART1
#else
 // #define USART_TX						GPIO_Pin_9
 // #define USART_RX						GPIO_Pin_10
 // #define USART_PORT 					GPIOA
 // #define USART_IO_CLK			 		RCC_APB2Periph_GPIOA
 // #define USART_CHANNEL					USART1
 // #define USART_CLK 						RCC_APB2Periph_USART1
#endif

// ULTRASONIC PINS
#ifdef STM32F10X_HD
#else
 #define MULT1							GPIO_Pin_0
 #define MULT2							GPIO_Pin_1
 #define MULTPORT						GPIOC
 #define MULTCLK						RCC_APB2Periph_GPIOC
 #define MULT_CHANNEL_SELECT_A			GPIO_Pin_1
 #define MULT_CHANNEL_SELECT_B			GPIO_Pin_2
 #define MULT_CHANNEL_SELECT_PORT		GPIOA
 #define MULT_CHANNEL_SELECT_CLK		RCC_APB2Periph_GPIOA
 
 #define ULTRASONIC_SLAVE_OUTPUT		GPIO_Pin_6
 #define ULTRASONIC_SLAVE_PORT	 		GPIOA
 #define ULTRASONIC_SLAVE_CLK			RCC_APB2Periph_GPIOA
 #define ULTRASONIC_SLAVE_TIMER			TIM3
 #define ULTRASONIC_SLAVE_TIMER_CLK		RCC_APB1Periph_TIM3
 
 #define ULTRASONIC_MASTER_OUTPUT		GPIO_Pin_3
 #define ULTRASONIC_MASTER_OUTPUT_CLK	RCC_APB2Periph_GPIOA
 #define ULTRASONIC_MASTER_TIMER		TIM2
 #define ULTRASONIC_MASTER_TIMER_CLK	RCC_APB1Periph_TIM2
 #define ULTRASONIC_MASTERSLAVE_ITR		TIM_TS_ITR1
 
 #define ULTRASONIC_INPUT				GPIO_Pin_7
 #define ULTRASONIC_INPUT_CLK			RCC_APB2Periph_GPIOA
 #define ULTRASONIC_INPUT_PORT			GPIOA
 #define ULTRASONIC_INPUT_ADC_PORT		ADC1
 #define ULTRASONIC_INPUT_ADC_CLK		RCC_APB2Periph_ADC1
 #define ULTRASONIC_INPUT_ADC_CHANNEL	ADC_Channel_7
#endif

#endif

