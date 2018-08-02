/**
  ******************************************************************************
  * @file    $FILE$.h
  * @author  LIAO Qinghai
  * @version V0.0.0
  * @date    27-April-2018
  * @brief   
  *
@verbatim
 ===============================================================================
                        ##### How to use this driver #####
 ===============================================================================
 

@endverbatim
           
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 LIAO Qinghai</center></h2>
  *
  ******************************************************************************  
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_IO_H
#define __BSP_IO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define LED_ON					(0)
#define LED_OFF					(1)
#define LED2					( PBout(13) )	
#define LED3					( PBout(14) )
#define LED2_Toggle				PBout(13) = !PBin(13);  
#define LED3_Toggle				PBout(14) = !PBin(14);  

	 
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/  


void BSP_IoInit(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_IO_H */


/************************ (C) COPYRIGHT LIAO Qinghai *****END OF FILE****/
