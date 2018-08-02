#include "bsp.h"
#include "bsp_delay.h"
#include "bsp_io.h"
#include "bsp_timer.h"
#include "bsp_can.h"
#include "bsp_encoder.h"
#include "bsp_eps.h"
#include "bsp_usart.h"
#include "bsp_ultrasonic.h"
#include "bsp_time.h"
#include "pin_configuration.h"

#include <math.h>

enum MCO_DEBUG_MODE{NONE,SYSCLK,HSI,HSE,PLLCLK_DIV2};

extern uint32_t start1,finish1, delta1, start2,finish2, delta2;
extern int rising1,rising2, transmitFlag1, transmitFlag2;

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

/*  ONLY FOR DEBUG
*   This func is used to configure MCO which can help debug clock
*   The selected Clock will output to PA.8 (Encoder#1+)
*/
void MCO(enum MCO_DEBUG_MODE mode)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //this function is only valid for STM32F0x
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //this function is only valid for STM32F0x
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//this function is only valid for STM32F0x
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);
	//RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1);  // HSE=8M --> PA.8
	//RCC_MCO1Config(RCC_MCO1Source_PLLCLK,RCC_MCO1Div_4);  // PLL=168M, 168/4=42M -->PA.8
	
	// pick one of the clocks to spew
	switch(mode){
		case NONE:
			RCC_MCOConfig(RCC_MCO_NoClock); 
			break;
		case SYSCLK:
			RCC_MCOConfig(RCC_MCO_SYSCLK); // Put on MCO pin the: System clock selected
			break;
		case HSI:
			RCC_MCOConfig(RCC_MCO_HSI); // Put on MCO pin the: freq. of high-speed internal 16 MHz RC oscillator
			break;
		case HSE:
			RCC_MCOConfig(RCC_MCO_HSE); // Put on MCO pin the: freq. of external crystal i.e 4-26 MHz
			break;
		case PLLCLK_DIV2:
			RCC_MCOConfig(RCC_MCO_PLLCLK_Div2);
			break;
		default:
			RCC_MCOConfig(RCC_MCO_NoClock); // Put on MCO pin the: System clock selected
			break;
	};
}

/**@brief watch dog
 *
 * @param prer
 * @param rlr
 */
void IWDG_Init(u8 prer,u16 rlr) 
{	
 	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //Ê¹ÄÜ¶Ô¼Ä´æÆ÷IWDG_PRºÍIWDG_RLRµÄÐ´²Ù×÷
	IWDG_SetPrescaler(prer);  //ÉèÖÃIWDGÔ¤·ÖÆµÖµ:ÉèÖÃIWDGÔ¤·ÖÆµÖµÎª64
	IWDG_SetReload(rlr);  //ÉèÖÃIWDGÖØ×°ÔØÖµ
	IWDG_ReloadCounter();  //°´ÕÕIWDGÖØ×°ÔØ¼Ä´æÆ÷µÄÖµÖØ×°ÔØIWDG¼ÆÊýÆ÷
	IWDG_Enable();  //Ê¹ÄÜIWDG	
}

/** @brief steering Controller
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
	
	Systick_Init();
	BSP_Init();
	BSP_DelayInit();	         		// Init delpy parameters
	BSP_LEDTestInit();       	    // Init LED on board to control rely	
	//MCO(NONE);                  // Debug system clock, use it only when you want to check clock
	BSP_CanInit(250);           	// Init CAN1, baudrate=250Kbps
	BSP_EncoderInit();						
	BSP_EpsInit();              	// Init RS485
	//BSP_UsartInit(115200);     	// Init USART1 for debug, baudrate=115200
	BSP_UltrasonicInit();				
	
	delay_ms(500);
	BSP_TimerInit(kEncoderFrequency);
	
	IWDG_Init(4, 120);	// 192ms
	IWDG_ReloadCounter();
	
	// flash out
	BSP_EncoderRead();
	BSP_EncoderRead();
	BSP_EncoderRead();
	
	printf("all initialized  \n");
	
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
		
		
	
	
		//Ultrasonic Routine
		for(int j=1; j<5; j++){
			selectChannel_Mux1(j); 
			selectChannel_Mux2(j); 
			ULTRASONIC_TRIGPORT->BRR  = ULTRASONIC_TRIG;
			delay_us(5);
			for(int x=0; x <8; x++){
				ULTRASONIC_TRIGPORT->BSRR=ULTRASONIC_TRIG;
				delay_us(10);
				ULTRASONIC_TRIGPORT->BRR=ULTRASONIC_TRIG;
				delay_us(10);
			}
			printf("ultrasonic sent  \n");
			delay_us(100);
			if(transmitFlag1 == 0){
				transmitFlag1+=1;
			}
			if(transmitFlag2 == 0){
				transmitFlag2+=1;
			}
			delay_ms(34);
			printf("Timestamp : %u  \n", micros());
		}
	}
	

	return 0;
}
