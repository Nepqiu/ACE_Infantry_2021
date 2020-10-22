#ifndef __RM35_INPUT_H
#define __RM35_INPUT_H

#include "RM35_Input.h"
#include "sys.h"

typedef struct Chassis_Input_Data					//处理输入值
{
	int16_t aim_data;				//实时遥控数据
	int16_t last_aim_data;				//上一次遥控数据	
	int16_t direction;			//实际运动方向（正为正，-为反）
	signed long aim_input;		//叠加的目标值
	signed long old_aim_input;		//上一叠加的目标值
	int16_t InputValue;
}Input;

extern Input CM_Input[4];		//定义遥控值

void Input_Init(void);			// 遥控值初始化函数
void Chassis_Re_Deal(void);
void Deal_InputPostion(void);		// 处理输入信号


#endif

