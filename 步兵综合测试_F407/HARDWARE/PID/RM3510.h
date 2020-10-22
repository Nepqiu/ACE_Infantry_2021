#ifndef __RM3510_H
#define __RM3510_H
#include "sys.h"

typedef struct Chassis_RM3510_Data						//3510电机码盘信息结构体
{
	int16_t go_ahead[2];		// 前进状态标志
	int16_t go_back[2];			// 后退状态标志
	int16_t NowAngle;      //当前机械角
	int16_t direction;			//实际运动方向（正为正，-为反）
	signed long cardinal_num;		//码盘基数
	signed long SumPostion;		//机械角差叠加值
	signed long Old_SumPostion;		//上个机械角差叠加值
}RM35;

extern RM35 CM_Data[4];			//3510电机码盘信息结构体

void RM35_Data_Init(void);				// 初始换函数
void RM35_Data_Deal(RM35* CM_Data);	// 3510码盘处理

#endif
