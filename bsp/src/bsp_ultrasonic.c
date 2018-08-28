#include "bsp_ultrasonic.h"
#include "pin_configuration.h"
#include "bsp_time.h"
#include <stdio.h>
#include "PGA460_USSC.h"
void InitPGA460() //���������ź�֮��2ms����֮��ͻ����õ���echo�ź�
	                //���ǵ������ź���40Khz�����ٷ��ķ������ϣ�����14��pulse�õ���echo�ź��бȽϴ��echo�ź�
{
	double result;
	short i;
	byte uartAddress = 0x00;
	initSTM32F1PGA460(0,19200);//��ʼ��ʹ��uartͨ�ţ�baud=115200�����ڵ�ַΪ0��<0-7>,
																//������ڵ�ַ��PGA�ĵ�ַ����¼������EEPROM�С�
																//����������ж�ȡ�����PGA�ĵ�ַ�����������PGA��UART�ĵ�ַΪ��������Ҫ�ĵ�ַ
	#if  SetUltraAddress
		SetPGAAddress(0x00);   //set Device address   , wehn you initallize the address ,this function should be commented 
	#endif
	
	for(i = 0; i<UltraDevNum;i++)  
	{
		initThresholds(1,i);//50% ����ֵ����Ӧ�����ֲ�18ҳ��Threshold Data Assignment,�����
										//����ת���������źŵıȽ���ֵ
		defaultPGA460(2,uartAddress); //���üĴ�����ǰ42����Ĭ�ϲ���
		initTVG(2,1,i);  //����ģ��ǰ�˵�ʱ�����棬������������
    
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
