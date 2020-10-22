/**
  ******************************************************************************
  * @file       Task_Gimbal.c/h
  * @brief      �����̨��������
  ******************************************************************************
  */

#ifndef __TASK_GIMBAL_H
#define __TASK_GIMBAL_H
#include "main.h"  
#include "pid.h"
#include "RemoteControl.h"


/*OS�������������Լ�����ʱ��*/
#define GIMBAL_TASK_INIT_TIME 	1
#define GIMBAL_CONTROL_TIME_MS  2


/* ��̨ģʽ */
typedef enum
{
	GIMBAL_STOP,         //ֹͣ
	
	GIMBAL_INITIALIZE,   //��ʼ��״̬
	GIMBAL_WORKING,      //�ֶ�״̬
	
	GIMBAL_AUTOATTACK,   //����״̬
	GIMBAL_AUTOBUFF,     //���״̬
//	GIMBAL_REPLENISHMEN, //����״̬
	
	GIMBAL_DOUBLE_GIMBAL,//˫��̨״̬������̨���飬����̨������ 
	
	GIMBAL_POWEROFF,     //����״̬
} Gimbal_mode_e;


typedef struct  //���������������
{
    PidTypeDef fire_pid;  //pid
	
	int8_t init_flag;    //��ʼ���ɹ���־
	
	int16_t output;
	
} gimbal_fire_control_t;

typedef struct  //��������̨�������
{
	Gimbal_mode_e sec_gimbal_mode;
	
    PidTypeDef second_yaw_pid;  //pid

	int8_t init_flag;           //��ʼ���ɹ���־
	int8_t photoelectric_zero;  //��Y����ֵ����־
	
	int16_t Yaw_different_angle;       //����̨�������̨�Ĳ��
	int16_t chassis_different_angle;   //����̨��Ե��̵Ĳ��
	
	int16_t output;
	
} gimbal_second_control_t;

typedef struct  //����pitch��������
{
//	Gimbal_mode_e gimbal_pitch_mode;
	
    PidTypeDef pitch_p_pid;  //pid
	PidTypeDef pitch_s_pid;  //pid
	
	int8_t init_flag;    //��ʼ���ɹ���־
	
	first_order_filter_type_t LowFilt_Pitch_Data;    //P���ͨ�˲���
	sliding_mean_filter_type_t Slidmean_Pitch_Data;  //P�Ử���˲���

	int16_t filt_output; //P���˲�ֵ
	
	int16_t output;
	
} gimbal_pitch_control_t;

typedef struct
{	
//	Gimbal_mode_e gimbal_yaw_mode;
	
	PidTypeDef yaw_p_pid;  //pid
	PidTypeDef yaw_s_pid;  //pid
	
	bool_t init_flag;           //Y���ʼ���ɹ���־
	int8_t photoelectric_zero;  //Y����ֵ����־
	
//	int16_t chassis_different_angle;  //��̨���̲��
	
	float angle;              //��̨��ǰ�Ƕ�
	float last_angle;         //��̨����Ƕ�

	int16_t filt_output;  //Y���˲�ֵ
	
	int16_t output;
	
} gimbal_yaw_control_t;


typedef struct
{
	const RC_ctrl_t *gimbal_RC; //����ʹ�õ�ң����ָ��
	
	gimbal_pitch_control_t pitch_c;
	gimbal_yaw_control_t yaw_c;
	
	Gimbal_mode_e fir_gimbal_mode;
	
	bool_t gimbal_all_flag;
	
} gimbal_first_control_t;



//���������������
extern gimbal_fire_control_t fire_control;
//��������̨�������
extern gimbal_second_control_t sec_gimbal_control;
//��������̨����
extern gimbal_first_control_t fir_gimbal_control;

//��̨��ʼ���ɹ���־
//extern int8_t INITIALIZE_flag;


/* ��̨������ */
extern void GIMBAL_Task(void *pvParameters);
/* ��̨��Ϊ */
extern void Gimbal_Work(gimbal_first_control_t *gimbal_working, int16_t gimbal_ch2, int16_t gimbal_ch3);
extern bool_t Gimbal_all_init_detection(void);

#endif
