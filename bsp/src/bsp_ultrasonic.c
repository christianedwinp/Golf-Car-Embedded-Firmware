#include "bsp_ultrasonic.h"
#include "pin_configuration.h"
#include "bsp_time.h"
#include <stdio.h>
#include "PGA460_USSC.h"
void InitPGA460() //发射驱动信号之后2ms左右之后就基本得到了echo信号
	                //我们的驱动信号是40Khz，看官方的仿真资料，发射14个pulse得到的echo信号有比较大的echo信号
{
	double result;
	short i;
	byte uartAddress = 0x00;
	initSTM32F1PGA460(0,19200);//初始化使用uart通信，baud=115200，串口地址为0，<0-7>,
																//这个串口地址是PGA的地址，记录在它的EEPROM中。
																//在这个函数中读取了这个PGA的地址并设置了这个PGA的UART的地址为我们所需要的地址
	#if  SetUltraAddress
		SetPGAAddress(0x00);   //set Device address   , wehn you initallize the address ,this function should be commented 
	#endif
	
	for(i = 0; i<UltraDevNum;i++)  
	{
		initThresholds(1,i);//50% 的阈值，对应数据手册18页的Threshold Data Assignment,这个是
										//经过转换后数字信号的比较阈值
		defaultPGA460(2,uartAddress); //设置寄存器的前42个的默认参数
		initTVG(2,1,i);  //设置模拟前端的时变增益，用来提高信噪比
    
//	registerWrite(0x4b,0x80);
		result = runDiagnostics(1,0,i);
		printf("frequency: %f Khz\n",result);
			result = runDiagnostics(0,1,i);
		printf("decay preiod: %f us\n",result);
			delay_ms(50);
		byte burnStat = burnEEPROM(i);
			if(burnStat == true){printf("EEPROM programmed successfully.");}
		else{printf("EEPROM program failed.");}
		
		uartAddress = uartAddress +0x01;
	}
}
