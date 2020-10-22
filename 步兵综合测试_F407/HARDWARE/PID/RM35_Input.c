#include "main.h"


Input CM_Input[4];		//定义遥控值

void Input_Init(void)								// 遥控值初始化函数
{
	int i;
	for(i=0;i<4;i++)
	{
		CM_Input[i].old_aim_input=CM_Input[i].aim_input=0;		
		CM_Input[i].InputValue = CM_Input[i].direction=0;
		CM_Input[i].last_aim_data = CM_Input[i].aim_data;
	}
}

//麦轮运动
void Chassis_Re_Deal(void)			//遥控控制底盘输入处理
{
	float ch0,ch1,ch2;
	
	ch0 = Control_data.RC_ch0;
	ch1 = Control_data.RC_ch1;
	ch2 = Control_data.RC_ch2;
	
	ch0 = float_limit(ch0,660,-660);
	ch1 = float_limit(ch1,660,-660);
	ch2 = float_limit(ch2,660,-660);
	//面对电机（带转轴侧），顺时针为正，逆时针为负，ch1代表前进，ch0代表右（负为左），ch2代表逆时针旋转（）		
	CM_Input[0].aim_data = (ch1 + ch0 + ch2)*Gain_speed;
	CM_Input[1].aim_data = (-ch1 + ch0 + ch2)*Gain_speed;
	CM_Input[2].aim_data = (ch1 - ch0 + ch2)*Gain_speed;
	CM_Input[3].aim_data = (-ch1 - ch0 + ch2)*Gain_speed;
	
}

void Deal_InputPostion(void)		// 处理输入信号
{
	Select_Control_Mode();						// 输入模式选择
}

