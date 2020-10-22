#ifndef __RM3510_H
#define __RM3510_H
#include "sys.h"

typedef struct Chassis_RM3510_Data						//3510���������Ϣ�ṹ��
{
	int16_t go_ahead[2];		// ǰ��״̬��־
	int16_t go_back[2];			// ����״̬��־
	int16_t NowAngle;      //��ǰ��е��
	int16_t direction;			//ʵ���˶�������Ϊ����-Ϊ����
	signed long cardinal_num;		//���̻���
	signed long SumPostion;		//��е�ǲ����ֵ
	signed long Old_SumPostion;		//�ϸ���е�ǲ����ֵ
}RM35;

extern RM35 CM_Data[4];			//3510���������Ϣ�ṹ��

void RM35_Data_Init(void);				// ��ʼ������
void RM35_Data_Deal(RM35* CM_Data);	// 3510���̴���

#endif
