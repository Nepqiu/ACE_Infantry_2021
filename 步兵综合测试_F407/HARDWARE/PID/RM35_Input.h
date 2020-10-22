#ifndef __RM35_INPUT_H
#define __RM35_INPUT_H

#include "RM35_Input.h"
#include "sys.h"

typedef struct Chassis_Input_Data					//��������ֵ
{
	int16_t aim_data;				//ʵʱң������
	int16_t last_aim_data;				//��һ��ң������	
	int16_t direction;			//ʵ���˶�������Ϊ����-Ϊ����
	signed long aim_input;		//���ӵ�Ŀ��ֵ
	signed long old_aim_input;		//��һ���ӵ�Ŀ��ֵ
	int16_t InputValue;
}Input;

extern Input CM_Input[4];		//����ң��ֵ

void Input_Init(void);			// ң��ֵ��ʼ������
void Chassis_Re_Deal(void);
void Deal_InputPostion(void);		// ���������ź�


#endif

