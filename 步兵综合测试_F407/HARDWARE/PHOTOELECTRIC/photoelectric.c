#include "photoelectric.h"

/*����̨yaw����ֵ���*/
void Yaw_Zero_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*����̨yaw����ֵ���*/
void Yaw_Two_Zero_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}


/*
* ���ܣ��������̨Y����ֵ��緵��ֵ
* ���룺��
* �������緵��ֵ1��0������ֵ����1������ֵ����0��
*/
u8 Yaw_Zero_Value(void)   //���Y���Ƿ�λ   ����ֵ����0�����򷵻�1
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);
}

/*
* ���ܣ���⸱��̨Y����ֵ��緵��ֵ
* ���룺��
* �������緵��ֵ1��0������ֵ����1������ֵ����0��
*/
u8 Sec_Yaw_Zero_Value(void)   //���Y���Ƿ�λ   ����ֵ����0�����򷵻�1
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2);
}

