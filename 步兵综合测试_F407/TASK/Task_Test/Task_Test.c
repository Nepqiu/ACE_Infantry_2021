#include "Task_Test.h"
#include "CAN_Receive.h"
#include "RemoteControl.h"
#include "pid.h"
#include "rmmotor.h"


PidTypeDef Motor_2006_test_PID;


float test_motor_2006 = 0.0f;
float test_2006_pid_P_out_jscope;
float test_2006_pid_I_out_jscope;
float test_2006_pid_D_out_jscope;


void Motor_2006_fire_test(void);
void Motor_2006_speed_test(void);
	

/* ���ڲ��Ը��ֵ�� */
void TEST_Task(void *pvParameters)
{
	
    while (1)
    {
		//���ң������ֵ�Ƿ���ȷ
		RC_data_is_error();
		
		
//		Motor_2006_speed_test();  //ch1���µ��ٶ�
		Motor_2006_fire_test();   //ֱ�ӿ����ٶ�
        vTaskDelay(2);
    }
}

/* �ǵðѲ�������ĺ궨����ϣ������0x207 */
void Motor_2006_speed_test(void)
{
	test_motor_2006 += (rc_ctrl.rc.ch[1]) * 0.02f;
	float_limit(test_motor_2006, 2000, -2000);
	
	Rmmotor_Speed_control(&Motor_2006_test_PID, test_motor_2006, motor_fire.speed, 2000);
	
	CAN1_Chassis_Gimbal_Fire(0, 0, Motor_2006_test_PID.out); 
}

/* �͵���һ������ */
void Motor_2006_fire_test(void)
{
	fp32 outttt;
	
	outttt = Rmmotor_Speed_control(&Motor_2006_test_PID, ((rc_ctrl.rc.ch[1])*5), motor_fire.speed, 2000);
	CAN1_Chassis_Gimbal_Fire(0, 0, /*((rc_ctrl.rc.ch[1])*25)*/Motor_2006_test_PID.out); //Motor_2006_test_PID.out        ((rc_ctrl.rc.ch[1])*25)
	
//	if(rc_ctrl.rc.ch[1] == 0)
//	{
////		Rmmotor_Speed_control(&Motor_2006_test_PID, 0, motor_fire.speed, 2000);
//		CAN1_Chassis_Gimbal_Fire(0, 0, 0);
//		
//	}
}
