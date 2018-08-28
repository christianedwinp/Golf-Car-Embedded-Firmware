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
#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bsp.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
	uint8_t is_auto;
	uint8_t bms_heart;
	uint8_t status;
	uint8_t cmd0;
	uint8_t cmd1;
	uint8_t cmd2;
	uint8_t cmd3;
	uint8_t sas_status;
	int16_t cmd_targetAngle;
	int16_t sas_angleVelocity;
	float sas_angle;
}BSP_CanMsg;

/* Exported constants --------------------------------------------------------*/
extern volatile BSP_CanMsg gCanMsg;

/* Exported macro ------------------------------------------------------------*/
#define BSP_CAN_UPDATE_COMMAND			(0x01)
#define BSP_CAN_UPDATE_STEERING			(0x02)
#define BUMPER_CRASH                (0x02)
#define BUMPER_NO_CRASH							(0x01)
	 
/* Exported functions --------------------------------------------------------*/  
void BSP_CanInit(uint32_t baud);
uint8_t BSP_CanSend(uint32_t id, uint8_t* msg,uint8_t len);	 
void BSP_CanSendEncoder(int16_t* data);
void BSP_CanSendBmsHeart(void);
void BSP_CanSendBumper(uint8_t Carsh);
#ifdef __cplusplus
}
#endif

#endif /* __BSP_CAN_H */


/************************ (C) COPYRIGHT LIAO Qinghai *****END OF FILE****/
