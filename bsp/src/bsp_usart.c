#include "bsp_usart.h"	
#include "bsp_eps.h"
#include "pin_configuration.h"
#include "string.h"
#include "bsp_time.h"
////////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)    
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
#endif

uint16_t UsartRxBuffSize = 50;
unsigned short Rx_Head = 0 ,Rx_Tail = 0;
char Rx[50];
static const uint8_t kUsartReceiveBufferSize = 64;
static uint8_t gUsartReceiveBuffer[kUsartReceiveBufferSize];


void BSP_UsartDmaInit(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		//使能DMA传输
	DMA_DeInit(DMA1_Channel5);  						 	//将DMA的通道4寄存器重设为缺省值

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;  		//DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&gUsartReceiveBuffer[0]; //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  					//数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = kUsartReceiveBufferSize;  			//DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  		//外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  				//内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 		//数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  							//工作在正常模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 					//DMA通道 x拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  							//DMA通道x没有设置为内存到内存传输
	
	DMA_Init(DMA1_Channel5, &DMA_InitStructure); 
	DMA_Cmd(DMA1_Channel5, ENABLE); 
} 



void BSP_UsartInit(uint32_t bound){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
//	BSP_UsartDmaInit();
	RCC_APB2PeriphClockCmd(USART_CLK | USART_IO_CLK, ENABLE); 
	
  	
	RCC_APB2PeriphClockCmd( TEST_LED_CLK, ENABLE); 
	
	
	GPIO_InitStructure.GPIO_Pin = USART_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART_PORT, &GPIO_InitStructure);    

	GPIO_InitStructure.GPIO_Pin = USART_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
	GPIO_Init(USART_PORT, &GPIO_InitStructure);   //初始化GPIOA

	USART_InitStructure.USART_BaudRate = 115200;				//波特率设置：115200
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;	//数据位数设置：8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 		//停止位设置：1位
	USART_InitStructure.USART_Parity = USART_Parity_Even ;  		//是否奇偶校验：无
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制模式设置：没有使能
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//接收与发送都使能
	
	USART_Init(USART_CHANNEL, &USART_InitStructure);  					//初始化USART1
	
	USART_ClearFlag(USART_CHANNEL, USART_FLAG_TC);
	//USART_ITConfig(USART_CHANNEL, USART_IT_IDLE, ENABLE);				//开启相关中断
//	USART_DMACmd(USART_CHANNEL,USART_DMAReq_Rx,ENABLE); 	
	USART_Cmd(USART_CHANNEL, ENABLE);

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;		//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;			//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);
}


void USART1_IRQHandler(void)                					
{	
	uint16_t len = 0;
	if(USART_GetITStatus(USART_CHANNEL, USART_IT_IDLE) != RESET)  
	  {
		USART_ReceiveData(USART_CHANNEL);			//读取数据 注意：这句必须要，否则不能够清除中断标志位
		len = kUsartReceiveBufferSize-DMA_GetCurrDataCounter(DMA1_Channel5); 

		BSP_EpsSendCmd(gUsartReceiveBuffer, len);
		USART_ClearITPendingBit(USART_CHANNEL, USART_IT_IDLE);         //清除中断标志
		DMA_Cmd(DMA1_Channel5, DISABLE); 
		DMA_SetCurrDataCounter(DMA1_Channel5,kUsartReceiveBufferSize);
		DMA_Cmd(DMA1_Channel5, ENABLE);                    		//恢复DMA指针，等待下一次的接收
	 } 
} 
 
void BSP_Usart2Init(int baud){//PA2 PA3
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(USART_CLK_US, ENABLE); 
	RCC_APB2PeriphClockCmd(USART_IO_CLK_US,ENABLE);
 
	
	GPIO_InitStructure.GPIO_Pin = USART_TX_US;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(USART_PORT_US, &GPIO_InitStructure);    

	GPIO_InitStructure.GPIO_Pin = USART_RX_US;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
	GPIO_Init(USART_PORT_US, &GPIO_InitStructure);   //初始化GPIOA

	USART_InitStructure.USART_BaudRate = baud;				//波特率设置：19200  2400 115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//数据位数设置：8位
	USART_InitStructure.USART_StopBits = USART_StopBits_2; 		//停止位设置：2位
	USART_InitStructure.USART_Parity = USART_Parity_No ;  		//是否奇偶校验：无
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制模式设置：没有使能
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//接收与发送都使能
	
	USART_Init(USART2, &USART_InitStructure);  					//初始化USART1
	
//	USART_ClearFlag(USART2, USART_FLAG_TC);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);				//开启接受中断	
	USART_Cmd(USART2, ENABLE);

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;			//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;		//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;			//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);
}

void USART2_IRQHandler(void)                					
{	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  
  {
		
	  unsigned short tempRx_Head = (Rx_Head+1)%UsartRxBuffSize;
		unsigned short tempRx_Tail = Rx_Tail;
		uint8_t data = USART_ReceiveData(USART2);			//读取数据 注意：这句必须要，否则不能够清除中断标志位

		if(tempRx_Head == tempRx_Tail)		
		{
			USART_ITConfig(USART2, USART_IT_RXNE, DISABLE); //接受缓存满了，停止接收数据
		}
		else{
		  Rx[Rx_Head] = data;
			Rx_Head = tempRx_Head;
		}
			
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);         //清除中断标志
	} 
} 

short Usart2_DataAvailable()
{
	uint16_t tempHead  = Rx_Head;
	uint16_t tempTail = Rx_Tail;
	short RxDelta = tempHead - tempTail;
	if(RxDelta<0)  RxDelta = UsartRxBuffSize + RxDelta;
	return RxDelta;
}
uint8_t Usart2_Getch()
{
	uint8_t ans;
	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
//	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	ans = Rx[Rx_Tail];
	Rx_Tail = (Rx_Tail+1)%UsartRxBuffSize;

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
//	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
	
	return ans;
}

void Usart_Flush()
{
	Rx_Tail = Rx_Head = 0;
	memset(Rx,0,sizeof(Rx));
}

void Usart2Send(unsigned char *Str, int len)
{
	int i;
	for(i=0;i<len;i++)
	{ 
		USART_SendData(USART2,Str[i]);
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);
		delay_us(100);
	
	}
}
int read(void){
	
	if(Usart2_DataAvailable())
	{
		return Usart2_Getch();
	}else
	{
		return 0;
	}

}
