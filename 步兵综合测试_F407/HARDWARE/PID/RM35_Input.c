#include "main.h"


Input CM_Input[4];		//����ң��ֵ

void Input_Init(void)								// ң��ֵ��ʼ������
{
	int i;
	for(i=0;i<4;i++)
	{
		CM_Input[i].old_aim_input=CM_Input[i].aim_input=0;		
		CM_Input[i].InputValue = CM_Input[i].direction=0;
		CM_Input[i].last_aim_data = CM_Input[i].aim_data;
	}
}

//�����˶�
void Chassis_Re_Deal(void)			//ң�ؿ��Ƶ������봦��
{
	float ch0,ch1,ch2;
	
	ch0 = Control_data.RC_ch0;
	ch1 = Control_data.RC_ch1;
	ch2 = Control_data.RC_ch2;
	
	ch0 = float_limit(ch0,660,-660);
	ch1 = float_limit(ch1,660,-660);
	ch2 = float_limit(ch2,660,-660);
	//��Ե������ת��ࣩ��˳ʱ��Ϊ������ʱ��Ϊ����ch1����ǰ����ch0�����ң���Ϊ�󣩣�ch2������ʱ����ת����		
	CM_Input[0].aim_data = (ch1 + ch0 + ch2)*Gain_speed;
	CM_Input[1].aim_data = (-ch1 + ch0 + ch2)*Gain_speed;
	CM_Input[2].aim_data = (ch1 - ch0 + ch2)*Gain_speed;
	CM_Input[3].aim_data = (-ch1 - ch0 + ch2)*Gain_speed;
	
}

void Deal_InputPostion(void)		// ���������ź�
{
	Select_Control_Mode();						// ����ģʽѡ��
}

