#include "main.h"
#include "CAN_receive.h"
/******************* 东莞理工学院ACE实验室 *******************/
int main(void)
{
	System_Init();          //系统初始化	
	StartTast();            //开始任务
	vTaskStartScheduler();  //开启任务调度
	
	while (1)
	{
//		CAN1_Chassis_Gimbal_Fire(500, 500, 500);
//		CAN1_Chassis_SetMsg(500, 500, 5000, 500);
	}
}
