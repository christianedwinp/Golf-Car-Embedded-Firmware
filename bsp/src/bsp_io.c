/**
  ******************************************************************************
  * @file    $FILE$.c
  * @author  LIAO Qinghai
  * @version V0.0.0
  * @date    27-April-2018
  * @brief   This file provides functions to configure,read and write GPIOs.
  *
@verbatim
 ===============================================================================
                        ##### How to use this driver #####
 ===============================================================================
 This module contains gpio manipulation: read(0/1) or write (0/1). 
 Usage:
	- Call Init function
	- Use the bit-band defined in the header file

@endverbatim
           
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 LIAO Qinghai</center></h2>
  *
  ******************************************************************************  
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_io.h"
#include "pin_configuration.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/**
* @brief  GPIO Init, 2 test LED
  * @param  
  * @note   
  * @retval None.
*/


void BSP_LEDTestInit(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( TEST_LED_CLK, ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin =  TEST_LED_1 | TEST_LED_2;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 				
	GPIO_SetBits(TEST_LED_PORT, TEST_LED_1 | TEST_LED_2);	 		 
}


/************************ (C) COPYRIGHT LIAO Qinghai *****END OF FILE****/
