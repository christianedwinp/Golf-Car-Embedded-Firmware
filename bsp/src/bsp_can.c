/**
  ******************************************************************************
  * @file    bsp_can.c
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

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
#include "bsp_eps.h"
#include "bsp_io.h"
#include "pin_configuration.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define  CAN_ID_SAS_DATA            (0x18F01D48)
#define  CAN_ID_BMS_DATA            (0x02F2)
#define  CAN_ID_BMS_CMD				(0x01F1)
#define  CAN_ID_LLC_CMD             (0x0380)
#define  CAN_ID_LLC_DATA            (0x0780)


#define SAS_ZERO					(-153)

static uint8_t CAN_BMS_BRAKE[8]={0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
const uint16_t kAngleMax = 600;
extern uint16_t gHeartbeatCnt;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile BSP_CanMsg gCanMsg;

/* Private function prototypes -----------------------------------------------*/
void BSP_CanParse(CanRxMsg* msg);

/* Private functions ---------------------------------------------------------*/


/**
  * @brief
  * @param
  * @note
  * @retval None.
  */
void BSP_CanInit(uint32_t baud)
{
//	assert_param(IS_BSP_CAN_BAUD(baud));
	GPIO_InitTypeDef        GPIO_InitStructure;
	CAN_InitTypeDef         CAN_InitStructure;
	CAN_FilterInitTypeDef   CAN_FilterInitStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;
	/*
	* APB1 42MHZ, baud = 42000/[(7+6+1)*(pre+1)]
	* 42000/(7+6+1) = 3000K,  pre=3000/baud - 1 (lib will -1)
	*/
	uint16_t pre = 12;      // 250Kbps
	if (3000 % baud == 0)
	{
		// if no reminder, use new pre
		pre = 3000 / baud;
	}
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | CAN_CLK, ENABLE);	                        											 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); 

	/* Configure CAN pin: RX */
	GPIO_InitStructure.GPIO_Pin = CAN_RX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	 // ��������
	GPIO_Init(CAN_PORT, &GPIO_InitStructure);
	/* Configure CAN pin: TX */  
	GPIO_InitStructure.GPIO_Pin = CAN_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // �����������
	GPIO_Init(CAN_PORT, &GPIO_InitStructure);
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

	/* CAN register init */
	CAN_DeInit(CAN1);	//������CAN��ȫ���Ĵ�������Ϊȱʡֵ
	CAN_StructInit(&CAN_InitStructure);//��CAN_InitStruct�е�ÿһ��������ȱʡֵ����

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM=DISABLE;		//û��ʹ��ʱ�䴥��ģʽ
	CAN_InitStructure.CAN_ABOM=ENABLE;		//û��ʹ���Զ����߹���
	CAN_InitStructure.CAN_AWUM=DISABLE;		//û��ʹ���Զ�����ģʽ
	CAN_InitStructure.CAN_NART=DISABLE;		//û��ʹ�ܷ��Զ��ش�ģʽ
	CAN_InitStructure.CAN_RFLM=DISABLE;		//û��ʹ�ܽ���FIFO����ģʽ
	CAN_InitStructure.CAN_TXFP=DISABLE;		//û��ʹ�ܷ���FIFO���ȼ�
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;//CAN����Ϊ����ģʽ
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq; 	//����ͬ����Ծ���1��ʱ�䵥λ
	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq; 	//ʱ���1Ϊ3��ʱ�䵥λ
	CAN_InitStructure.CAN_BS2=CAN_BS2_2tq; 	//ʱ���2Ϊ2��ʱ�䵥λ
	CAN_InitStructure.CAN_Prescaler = pre;  	//ʱ�䵥λ����Ϊ60	
	CAN_Init(CAN1,&CAN_InitStructure);		//������Ϊ��72M/2/12/(1+9+2)=250k
									 

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber=1;						//ָ��������Ϊ1
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//ָ��������Ϊ��ʶ������λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//������λ��Ϊ32λ
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;				//��������ʶ���ĸ�16λֵ
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;					//��������ʶ���ĵ�16λֵ
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;			//���������α�ʶ���ĸ�16λֵ
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;				//���������α�ʶ���ĵ�16λֵ
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;		//�趨��ָ���������FIFOΪ0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//ʹ�ܹ�����
	CAN_FilterInit(&CAN_FilterInitStructure);

	/* CAN FIFO0 message pending interrupt enable */ 
	CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE); //ʹ��FIFO0��Ϣ�Һ��ж�.

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	gCanMsg.status = 0;
	gCanMsg.bms_heart = 1;
	gCanMsg.is_auto = 0;
}

void BSP_CanParse(CanRxMsg* msg)
{
	int16_t angle = 0;
	uint32_t id = msg->IDE == CAN_Id_Standard ? msg->StdId : msg->ExtId;

	switch (id)
	{
	case CAN_ID_BMS_DATA:
		if ( msg->Data[4] & 0x01 )
		{
			if(gCanMsg.is_auto)
			{
				// release eps
				BSP_EpsRelease();
			}
			gCanMsg.is_auto = 0;
		}
		else
		{
			gCanMsg.is_auto = 1;
		}
		break;
	case CAN_ID_LLC_CMD:
		// once we receive this msg, don't send hert-beat anymore
		gCanMsg.bms_heart = 0;
		gHeartbeatCnt = 0;
		gCanMsg.cmd0 = msg->Data[0];
		gCanMsg.cmd1 = msg->Data[1];
		angle = (msg->Data[3] << 8) | msg->Data[2];
		if (angle > kAngleMax)
		{
			angle = kAngleMax;
		}
		if (angle < -kAngleMax)
		{
			angle = -kAngleMax;
		}
		gCanMsg.cmd_targetAngle = angle;
		gCanMsg.status |= BSP_CAN_UPDATE_COMMAND;
		break;
	case CAN_ID_SAS_DATA:
		gCanMsg.sas_status = msg->Data[3];
		gCanMsg.sas_angleVelocity = msg->Data[2];
		if(msg->Data[3]&0x10)
		{
			gCanMsg.sas_angleVelocity = -1*msg->Data[2];
		}
		angle = (msg->Data[1] << 8) | msg->Data[0];
		gCanMsg.sas_angle = -angle/10.0;
		gCanMsg.sas_angleVelocity  = -gCanMsg.sas_angleVelocity;
		gCanMsg.status |= BSP_CAN_UPDATE_STEERING;
		
		LED2_Toggle
		break;
	default:
		break;
	}
}


void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg msg;
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) == SET)
	{
		CAN_Receive(CAN1, CAN_FIFO0, &msg);
		BSP_CanParse(&msg);
	}
}


uint8_t BSP_CanSend(uint32_t id, uint8_t* msg, uint8_t len)
{
	uint8_t mbox = CAN_TxStatus_NoMailBox;
	uint8_t i = 0;
	// only sent std data frame
	if (len > 8)
	{
		len = 8;
	}
	CanTxMsg TxMessage;
	TxMessage.StdId = id;       // ��׼��ʶ��Ϊ0
	TxMessage.ExtId = id;       // ������չ��ʾ����29λ��
	if (id > 0x7FF)             // ʹ����չ��ʶ��
	{
		TxMessage.IDE = CAN_ID_EXT;
	}
	else
	{
		TxMessage.IDE = CAN_ID_STD;
	}
	TxMessage.RTR = CAN_RTR_DATA;          // ��Ϣ����Ϊ����֡��һ֡8λ
	TxMessage.DLC = len;        // ������֡��Ϣ
	for (i = 0; i < len; i++)
	{
		TxMessage.Data[i] = msg[i];
	}
	mbox = CAN_Transmit(CAN1, &TxMessage);

	return ( mbox == CAN_TxStatus_NoMailBox) ? 1 : 0;

	// here we don't wait
	//while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;   //�ȴ����ͽ���
}


void BSP_CanSendEncoder(int16_t* data)
{
	BSP_CanSend(CAN_ID_LLC_DATA, (uint8_t*)data, 8);	
}

void BSP_CanSendBmsHeart()
{
	BSP_CanSend(CAN_ID_BMS_CMD, CAN_BMS_BRAKE, 8);
}


/************************ (C) COPYRIGHT LIAO Qinghai *****END OF FILE****/
