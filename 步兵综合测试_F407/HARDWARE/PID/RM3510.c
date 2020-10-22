#include "main.h"

RM35 CM_Data[4];

void RM35_Data_Init(void)						// 初始化函数
{
	int i;
	for(i=0;i<4;i++)
	{
		CM_Data[i].cardinal_num=CM_Data[i].direction=0;
		CM_Data[i].go_ahead[0]=CM_Data[i].go_ahead[1]=CM_Data[i].go_back[0]=CM_Data[i].go_back[1]=0;
		CM_Data[i].Old_SumPostion=CM_Data[i].SumPostion=CM_Data[i].NowAngle;
	}
}

void RM35_Data_Deal(RM35* CM_Data)				// 3510码盘处理
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
	else if(nowangle > 7000)			//轮子为正转接近下个码盘警戒												
		CM_Data->go_ahead[0]=1;
	else													//轮子为反转接近下个码盘警戒	
		CM_Data->go_back[0]=1;
	
	if(CM_Data->go_ahead[0]==1 && nowangle < 2000)										//确定轮子为正转且进入下个码盘
		CM_Data->go_ahead[1]=1;
				
	if(CM_Data->go_back[0]==1 && nowangle > 7000)										//确定轮子为反转且进入下个码盘
		CM_Data->go_back[1]=1;
		
	//恢复前进标志位
	if(CM_Data->go_ahead[0]==1 && CM_Data->go_ahead[1]==1)
	{
		CM_Data->cardinal_num+=8192;
		CM_Data->go_ahead[0]=0;
		CM_Data->go_ahead[1]=0;
	}
	
	//恢复后退标志位
	if(CM_Data->go_back[0]==1 && CM_Data->go_back[1]==1)
	{
		CM_Data->cardinal_num-=8192;
		CM_Data->go_back[0]=0;
		CM_Data->go_back[1]=0;
	}
	
	// 防止所记圈数超过实际电机比例
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
	CM_Data->SumPostion = CM_Data->cardinal_num+nowangle;	// 当前的叠加机械角值
}
