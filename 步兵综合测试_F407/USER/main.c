#include "main.h"
#include "CAN_receive.h"
/******************* ��ݸ��ѧԺACEʵ���� *******************/
int main(void)
{
	System_Init();          //ϵͳ��ʼ��	
	StartTast();            //��ʼ����
	vTaskStartScheduler();  //�����������
	
	while (1)
	{
//		CAN1_Chassis_Gimbal_Fire(500, 500, 500);
//		CAN1_Chassis_SetMsg(500, 500, 5000, 500);
	}
}
