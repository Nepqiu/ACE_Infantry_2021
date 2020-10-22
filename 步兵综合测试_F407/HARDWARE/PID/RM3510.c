#include "main.h"

RM35 CM_Data[4];

void RM35_Data_Init(void)						// ��ʼ������
{
	int i;
	for(i=0;i<4;i++)
	{
		CM_Data[i].cardinal_num=CM_Data[i].direction=0;
		CM_Data[i].go_ahead[0]=CM_Data[i].go_ahead[1]=CM_Data[i].go_back[0]=CM_Data[i].go_back[1]=0;
		CM_Data[i].Old_SumPostion=CM_Data[i].SumPostion=CM_Data[i].NowAngle;
	}
}

void RM35_Data_Deal(RM35* CM_Data)				// 3510���̴���
{
	int16_t nowangle;
		
	nowangle = CM_Data->NowAngle;
	
	if(nowangle >=2000 && nowangle <= 7000)
	{
		CM_Data->go_ahead[0]=0;
		CM_Data->go_ahead[1]=0;
		CM_Data->go_back[0]=0;
		CM_Data->go_back[1]=0;
	}
	else if(nowangle > 7000)			//����Ϊ��ת�ӽ��¸����̾���												
		CM_Data->go_ahead[0]=1;
	else													//����Ϊ��ת�ӽ��¸����̾���	
		CM_Data->go_back[0]=1;
	
	if(CM_Data->go_ahead[0]==1 && nowangle < 2000)										//ȷ������Ϊ��ת�ҽ����¸�����
		CM_Data->go_ahead[1]=1;
				
	if(CM_Data->go_back[0]==1 && nowangle > 7000)										//ȷ������Ϊ��ת�ҽ����¸�����
		CM_Data->go_back[1]=1;
		
	//�ָ�ǰ����־λ
	if(CM_Data->go_ahead[0]==1 && CM_Data->go_ahead[1]==1)
	{
		CM_Data->cardinal_num+=8192;
		CM_Data->go_ahead[0]=0;
		CM_Data->go_ahead[1]=0;
	}
	
	//�ָ����˱�־λ
	if(CM_Data->go_back[0]==1 && CM_Data->go_back[1]==1)
	{
		CM_Data->cardinal_num-=8192;
		CM_Data->go_back[0]=0;
		CM_Data->go_back[1]=0;
	}
	
	// ��ֹ����Ȧ������ʵ�ʵ������
	if(CM_Data->cardinal_num<0)
	{
		CM_Data->cardinal_num=A_Round-8192;
		CM_Data->direction--;
	}
		
	else if(CM_Data->cardinal_num>A_Round-8192)
	{
		CM_Data->cardinal_num=0;
		CM_Data->direction++;
	}	
	CM_Data->SumPostion = CM_Data->cardinal_num+nowangle;	// ��ǰ�ĵ��ӻ�е��ֵ
}
