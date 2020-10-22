#include "SysInit.h"

////HARDWARE
#include "led.h"
#include "key.h"
#include "can.h"
#include "pid.h"
#include "time.h"
#include "iwdg.h"
#include "usart.h"
#include "photoelectric.h"

//REFEREE

//DEFINE
#include "Mode_Def.h"
#include "PID_Def.h"
#include "Speed_Def.h"

//CONTROL
#include "maths.h"
#include "filter.h"
#include "RemoteControl.h"
#include "CAN_receive.h"
#include "chassis_behaviour.h"
#include "IMU.h"
#include "rmmotor.h"

//Task
#include "Task_Start.h"
#include "Task_Remote.h"
#include "Task_Detect.h"
#include "Task_Chassis.h"
#include "Task_Gimbal.h"
#include "Task_Safecheck.h"
#include "Task_Test.h"


void System_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);  //����ϵͳ�ж����ȼ����� 4
	
	delay_init(180);  //��ʱ������ʼ��
	
	/* ���ڳ�ʼ�� */ 
	remote_control_init();     //����һ��ʼ����ң������ʼ����
//	IMU_control_init();        //���ڶ���ʼ�����������������ݣ�
	
	/* GPIO��ʼ�� */ 
//	LED_Init();               //����LED��ʼ��
	
	/* ����ʼ�� */
	Yaw_Zero_GPIO_Init();     //Y����ֵ����ʼ�� PA3
	Yaw_Two_Zero_GPIO_Init(); //��Y����ֵ����ʼ�� PA2
	
	/* CAN�ӿڳ�ʼ�� */
	CAN1_mode_init(CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_5tq, 3, CAN_Mode_Normal);
	CAN2_mode_init(CAN_SJW_1tq, CAN_BS1_8tq, CAN_BS2_5tq, 3, CAN_Mode_Normal);
	
	/* PID������ʼ�� */
	PID_Parameter_Init();
	
#ifdef watch_dog
     /* ���Ź���ʼ�� */
	IWDG_Init(IWDG_Prescaler_64, 320);   //��������   4�� 300
#endif
	
	delay_ms(100);
}


void PID_Parameter_Init(void)
{		
	//��ʼ�������ٶȻ�pid
	pid_init(&chassis_control.chassis_speed_pid[0], CHASSIS_MOTOR1_PID_Kp, CHASSIS_MOTOR1_PID_Ki, CHASSIS_MOTOR1_PID_Kd, 0, 0);
	pid_init(&chassis_control.chassis_speed_pid[1], CHASSIS_MOTOR2_PID_Kp, CHASSIS_MOTOR2_PID_Ki, CHASSIS_MOTOR2_PID_Kd, 0, 0);
	pid_init(&chassis_control.chassis_speed_pid[2], CHASSIS_MOTOR3_PID_Kp, CHASSIS_MOTOR3_PID_Ki, CHASSIS_MOTOR3_PID_Kd, 0, 0);
	pid_init(&chassis_control.chassis_speed_pid[3], CHASSIS_MOTOR4_PID_Kp, CHASSIS_MOTOR4_PID_Ki, CHASSIS_MOTOR4_PID_Kd, 0, 0);	 
	
	//�����ƶ�����pid
	pid_init(&CHASSIS_MOVE_FOLLOW_PID, CHASSIS_MOVE_FOLLOW_P, CHASSIS_MOVE_FOLLOW_I, CHASSIS_MOVE_FOLLOW_D, 0, 0);
	
	//������ת����pid
	pid_init(&CHASSIS_ROTATE_FOLLOW_PID, CHASSIS_ROTATE_FOLLOW_P, CHASSIS_ROTATE_FOLLOW_I, CHASSIS_ROTATE_FOLLOW_D, 0, 0);
	
	//Pitch
	pid_init(&fir_gimbal_control.pitch_c.pitch_p_pid, GIMBAL_P_PITCH_P, GIMBAL_P_PITCH_I, GIMBAL_P_PITCH_D, 0, 0);
	pid_init(&fir_gimbal_control.pitch_c.pitch_s_pid, GIMBAL_S_PITCH_P, GIMBAL_S_PITCH_I, GIMBAL_S_PITCH_D, 0, 0);
	
	//Yaw
	pid_init(&fir_gimbal_control.yaw_c.yaw_p_pid, GIMBAL_P_YAW_P, GIMBAL_P_YAW_I, GIMBAL_P_YAW_D, 0, 0);
	pid_init(&fir_gimbal_control.yaw_c.yaw_s_pid, GIMBAL_S_YAW_P, GIMBAL_S_YAW_I, GIMBAL_S_YAW_D, 0, 0);
	
	//���Ե��pid
	pid_init(&Motor_2006_test_PID, 8.0f, 0.0f, 0.2f, 0, 0);
}

