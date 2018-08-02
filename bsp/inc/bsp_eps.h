/**
  ******************************************************************************
  * @file    bsp_eps.h
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
#ifndef __BSP_EPS_H
#define __BSP_EPS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/  
void BSP_EpsInit	(void);
void BSP_EpsRelease	(void);
void BSP_EpsSet		(int16_t value);	
void BSP_EpsSendCmd	(const uint8_t* ptr, uint16_t len);	 
	 
#ifdef __cplusplus
}
#endif

#endif /* __BSP_EPS_H */


/************************ (C) COPYRIGHT LIAO Qinghai *****END OF FILE****/
