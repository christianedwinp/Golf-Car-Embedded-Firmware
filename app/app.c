#include "bsp.h"
#include "bsp_delay.h"
#include "bsp_io.h"
#include "bsp_timer.h"
#include "bsp_encoder.h"
#include "bsp_can.h"
#include "bsp_eps.h"
#include "bsp_usart.h"
#include "bsp_ultrasonic.h"
#include "pin_configuration.h"

#include <math.h>

extern volatile uint32_t ticks;
extern int ultrasonicProgState;
extern int muxNumber;
extern int muxChannel;
extern int ultrasonicNumber;
extern float distance[8];
extern uint32_t transmitTime, receiveTime, previousReceiveTime;

const uint8_t kEncoderFrequency = 20;
float kControllerP = 1.5;
float kControllerI = 0.3;
float kControllerD = 0;
uint16_t kControlSpeedMax = 1500;
uint16_t kControllerDeadzoneHigh = 20;
uint16_t kControllerDeadzoneLow = 15;
uint16_t kControllerDeadzoneLowLow = 5;
uint16_t kControllerDeadzone = 10;
float kControllerDeadzoneHighRatioR = 0.07;
float kControllerDeadzoneHighRatioL = 0.04;

const uint16_t kHeartbeatMax = 9;
uint16_t gHeartbeatCnt = 0;
static uint16_t velocity_flag = 0;

/*
*   ONLY FOR DEBUG
*   This func is used to configure MCO which can help debug clock
*   The selected Clock will output to PA.8 (Encoder#1+)
*/
/*
void MCO(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);
	//RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1);      // HSE=8M --> PA.8
	RCC_MCO1Config(RCC_MCO1Source_PLLCLK,RCC_MCO1Div_4);  // PLL=168M, 168/4=42M -->PA.8
}
*/



/**
 * @brief system ticker
 *
 * this function to get clock frequency of the board
 */
void systickInit(uint16_t frequency) {
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  (void) SysTick_Config(RCC_Clocks.HCLK_Frequency / frequency);
}

/**
 * @brief systick handler
 *
 * this function to update variable tick every milliseconds
 */
// void SysTick_Handler(void) {
//   ticks++;
// }

/**
 * @brief millis
 *
 * this function to get system clock in milliseconds
 */
static __INLINE uint32_t millis (void)
{
   return ticks;
}

/**
 * @brief watch dog
 *
 * @param 
 * @param 
 * @param 
 */
void IWDG_Init(u8 prer,u16 rlr) 
{	
 	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //Ê¹ÄÜ¶Ô¼Ä´æÆ÷IWDG_PRºÍIWDG_RLRµÄÐ´²Ù×÷
	IWDG_SetPrescaler(prer);  //ÉèÖÃIWDGÔ¤·ÖÆµÖµ:ÉèÖÃIWDGÔ¤·ÖÆµÖµÎª64
	IWDG_SetReload(rlr);  //ÉèÖÃIWDGÖØ×°ÔØÖµ
	IWDG_ReloadCounter();  //°´ÕÕIWDGÖØ×°ÔØ¼Ä´æÆ÷µÄÖµÖØ×°ÔØIWDG¼ÆÊýÆ÷
	IWDG_Enable();  //Ê¹ÄÜIWDG	
}

/**
 * @brief steering Controller
 *
 * @param target
 * @param current
 * @param velocity
 */
void steeringController(int16_t target, int16_t current, int16_t velocity)
{
	float error = target - current;
	int16_t speed = 0;
	
	if( error > kControllerDeadzone || error < -kControllerDeadzone )
	{
		speed = error*kControllerP;
		velocity_flag++;
	}
	else
	{
		kControllerDeadzone = kControllerDeadzoneHigh;
		if(target < -100)
		{
			kControllerDeadzone += fabs(target*kControllerDeadzoneHighRatioR);
		}
		else if(target > 100)
		{
			kControllerDeadzone += fabs(target*kControllerDeadzoneHighRatioL);
		}
	}
	
	if(speed > kControlSpeedMax)
	{
		speed = kControlSpeedMax;
	}
	else if(speed < -kControlSpeedMax)
	{
		speed = -kControlSpeedMax;
	}
	
	
	BSP_EpsSet(speed);
}

int main(void)
{
	uint8_t led3 = 0;
	int16_t last_angle_target = 0;

	systickInit(1000);
	BSP_Init();
	BSP_DelayInit();	         	// Init delpy parameters
	BSP_IoInit();       	     	// Init IO to control rely	
	//MCO();                     	// Debug system clock, use it only when you want to check clock
	BSP_CanInit(250);           	// Init CAN1, baudrate=250Kbps
	BSP_EncoderInit();
	BSP_EpsInit();              	// Init RS485
	BSP_UsartInit(115200);     	 	// Init USART1 for debug, baudrate=115200
	BSP_UltrasonicInit();			// Init Ultrasonic
	
	BSP_DelayMs(500);
	BSP_TimerInit(kEncoderFrequency);
	
	IWDG_Init(4, 120);	// 192ms
	IWDG_ReloadCounter();
	
	// flash out
	BSP_EncoderRead();
	BSP_EncoderRead();
	BSP_EncoderRead();
	
	while (1)
	{
		//update by timer
		if (gTimerFlag)
		{
			gTimerFlag = 0;
			BSP_EncoderRead();
			BSP_CanSendEncoder(gEncoder);
			
			if(gCanMsg.bms_heart)
			{
				BSP_CanSendBmsHeart();
			}
			else
			{
				gHeartbeatCnt++;
			}
			IWDG_ReloadCounter();
			
			// indicate status
			led3++;
			if(led3 >= 10)
			{
				LED3_Toggle
				led3 = 0;
			}
		}
		
		// timeout send heartbeat to keep alive
		if(gHeartbeatCnt > kHeartbeatMax)
		{
			gCanMsg.bms_heart = 1;
			gHeartbeatCnt = 0;
			BSP_CanSendBmsHeart();
		}
				
		if (gCanMsg.is_auto && gCanMsg.status )
		{
			if((gCanMsg.status & BSP_CAN_UPDATE_COMMAND ) && fabs(gCanMsg.cmd_targetAngle-last_angle_target) > 3)
			{
				// small angle -> small deadzone
				if(fabs(gCanMsg.cmd_targetAngle) < 50)
				{
					kControllerDeadzone = kControllerDeadzoneLowLow;
				}
				else
				{
					kControllerDeadzone = kControllerDeadzoneLow;
				}
			}
			last_angle_target = gCanMsg.cmd_targetAngle;
			steeringController(gCanMsg.cmd_targetAngle, gCanMsg.sas_angle,
			                   gCanMsg.sas_angleVelocity);
			gCanMsg.status = 0;
		}

		//ULTRASONIC ROUTINE
		if(ultrasonicProgState == 0 && ADC_GetConversionValue(ULTRASONIC_INPUT_ADC_PORT) > ULTRASONIC_THRESHOLD){
			ultrasonicProgState++;
		}
		else if(ultrasonicProgState == 1 && ADC_GetConversionValue(ULTRASONIC_INPUT_ADC_PORT) < ULTRASONIC_THRESHOLD){
			ultrasonicProgState++;
			//record start time
			transmitTime = millis();		
		}else if(ultrasonicProgState == 2 && ADC_GetConversionValue(ULTRASONIC_INPUT_ADC_PORT) > ULTRASONIC_THRESHOLD){
			ultrasonicProgState++;
			//record echo time
			receiveTime = millis();
		}		
		

		// printf("\r\n ADC value on ch%d = %d\r\n",
  //           index, (uint16_t)((ADC_values[index]+ADC_values[index+8]
  //                   +ADC_values[index+16]+ADC_values[index+24])/4));

	}
	

	return 0;
}
