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
	

/* 用于测试各种电机 */
void TEST_Task(void *pvParameters)
{
	
    while (1)
    {
		//检查遥控器数值是否正确
		RC_data_is_error();
		
		
//		Motor_2006_speed_test();  //ch1上下调速度
		Motor_2006_fire_test();   //直接控制速度
        vTaskDelay(2);
    }
}

/* 记得把拨弹电机的宏定义加上，电调：0x207 */
void Motor_2006_speed_test(void)
{
	test_motor_2006 += (rc_ctrl.rc.ch[1]) * 0.02f;
	float_limit(test_motor_2006, 2000, -2000);
	
	Rmmotor_Speed_control(&Motor_2006_test_PID, test_motor_2006, motor_fire.speed, 2000);
	
	CAN1_Chassis_Gimbal_Fire(0, 0, Motor_2006_test_PID.out); 
}

/* 和底盘一样控制 */
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
