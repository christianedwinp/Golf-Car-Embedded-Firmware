#include "bsp_ultrasonic.h"
#include "pin_configuration.h"
#include "bsp_time.h"
#include <stdio.h>
#include "PGA460_USSC.h"
void InitPGA460() //���������ź�֮��2ms����֮��ͻ����õ���echo�ź�
	                //���ǵ������ź���40Khz�����ٷ��ķ������ϣ�����14��pulse�õ���echo�ź��бȽϴ��echo�ź�
{
	
	initSTM32F1PGA460(0,115200,0);//��ʼ��ʹ��uartͨ�ţ�baud=115200�����ڵ�ַΪ0��<0-7>,
																//������ڵ�ַ��PGA�ĵ�ַ����¼������EEPROM�С�
																//����������ж�ȡ�����PGA�ĵ�ַ�����������PGA��UART�ĵ�ַΪ��������Ҫ�ĵ�ַ
	defaultPGA460(2); //���üĴ�����ǰ42����Ĭ�ϲ���
	initThresholds(1);//50% ����ֵ����Ӧ�����ֲ�18ҳ��Threshold Data Assignment,�����
										//����ת���������źŵıȽ���ֵ
	initTVG(2,1);  //����ģ��ǰ�˵�ʱ�����棬������������

	byte burnStat = burnEEPROM();
	if(burnStat == true){printf("EEPROM programmed successfully.");}
	else{printf("EEPROM program failed.");}

}
