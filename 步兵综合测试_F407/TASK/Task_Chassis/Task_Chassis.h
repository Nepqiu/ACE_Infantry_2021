/**
  ******************************************************************************
  * @file       Task_Chassis.c/h
  * @brief      ��ɵ��̿�������
  ******************************************************************************
  */
#ifndef __TASK_CHASSIS_H
#define __TASK_CHASSIS_H
#include "main.h"
#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "maths.h"
#include "pid.h"

/*OS�������������Լ�����ʱ��*/
#define CHASSIS_TASK_INIT_TIME  5
#define CHASSIS_CONTROL_TIME    2


/*����ģʽ*/
typedef enum
{
	CHASSIS_STOP,        //ֹͣ
	CHASSIS_INITIALIZE,  //��ʼ����
	CHASSIS_POWEROFF,    //����
	CHASSIS_WORKING,     //����
	
	CHASSIS_FOLLOW,      //����
	CHASSIS_NO_FOLLOW,   //������
	
	CHASSIS_TWIST_WAIST, //Ť��
	CHASSIS_ROTATION,    //С����
} Chassis_mode_e;


typedef struct
{
    fp32 speed;
    fp32 speed_set;
	
    int16_t output;
} Motor_t;


typedef struct //�����������ݽṹ��
{
	const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��
	
	Motor_t chassis_motor[4];           //���̵������(�������ͳһ�ṹ��ָ��)
	PidTypeDef chassis_speed_pid[4];    //pid

	Chassis_mode_e chassis_mode;        //���̿���״̬��
	Chassis_mode_e last_chassis_mode;   //�����ϴο���״̬��
	
	first_order_filter_type_t LowFilt_chassis_vx;  //��ͨ�˲���
	first_order_filter_type_t LowFilt_chassis_vy;  //��ͨ�˲���

//	fp32 speed_x;                         //�����ٶ� ǰ������ ǰΪ������λ m/s
//	fp32 speed_y;                         //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
//	fp32 speed_z;                         //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	fp32 speed_x_set;                     //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
	fp32 speed_y_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
	fp32 speed_z_set;                     //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	
	fp32 chassis_gimbal_angel;    //��������̨�ĽǶ�
	
//	fp32 max_speed_x;   //ǰ����������ٶ� ��λm/s
//	fp32 min_speed_x;   //ǰ��������С�ٶ� ��λm/s
//	fp32 max_speed_y;   //���ҷ�������ٶ� ��λm/s
//	fp32 min_speed_y;   //���ҷ�����С�ٶ� ��λm/s
	
//	fp32 chassis_yaw;   //�����Ǻ���̨������ӵ�yaw�Ƕ�
//	fp32 chassis_pitch; //�����Ǻ���̨������ӵ�pitch�Ƕ�
//	fp32 chassis_roll;  //�����Ǻ���̨������ӵ�roll�Ƕ�
} chassis_control_t;




//���̿������� static
extern chassis_control_t  chassis_control; 

//�����ƶ�����PID
extern PidTypeDef CHASSIS_MOVE_FOLLOW_PID; 
//������ת����PID
extern PidTypeDef CHASSIS_ROTATE_FOLLOW_PID;


//����������
extern void CHASSIS_Task(void *pvParameters);
//���̿���������	
extern void chassis_set_remote(int16_t ch0, int16_t ch1, int16_t ch2);

#endif
