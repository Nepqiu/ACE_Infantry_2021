/**
  ******************************************************************************
  * @file       Task_Start.c/h
  * @brief      ��ʼ����
  ******************************************************************************
  */
  
#include "Task_Start.h"
#include "Task_Detect.h"
#include "Task_Chassis.h"
#include "Task_Gimbal.h"
#include "Task_Safecheck.h"
#include "Task_Remote.h"
#include "Task_Test.h"


//WorkStatus work_st = POWEROFF;  //Ĭ�Ϲ���ģʽΪ����ģʽ
WorkStatus work_st = WORKING;


/* ��̬�ǽ������� */
//#define INS_TASK_PRIO  5
//#define INS_TASK_SIZE  200
//static TaskHandle_t INSTask_Handler;

/* ��̨�������� */
#define GIMBAL_TASK_PRIO  6
#define GIMBAL_STK_SIZE   200
TaskHandle_t GIMBALTask_Handler;

/* ���̿������� */
#define Chassis_TASK_PRIO  6
#define Chassis_STK_SIZE   200
TaskHandle_t ChassisTask_Handler;

/* ң�����ݴ������ */ 
#define REMOTE_TASK_PRIO  9
#define REMOTE_STK_SIZE  150
TaskHandle_t RemoteTask_Handler;

/* �������� */
#define Detect_TASK_PRIO  10
#define Detect_STK_SIZE   50
static TaskHandle_t DetectTask_Handler;

/* ��ȫ������ */
#define SAFE_TASK_PRIO  4
#define SAFE_STK_SIZE   80
static TaskHandle_t SafecheckTask_Handler;

/* ������Գ��� */
#define TEST_TASK_PRIO  5
#define TEST_STK_SIZE   200
static TaskHandle_t TestTask_Handler;

/* ��ʼ���� */
#define START_TASK_PRIO  1
#define START_STK_SIZE   50
static TaskHandle_t StartTask_Handler;



void START_Task(void *pvParameters)
{
	taskENTER_CRITICAL();  //Ϊ�˱�֤��PORTA�Ĵ����ķ��ʲ����жϣ������ʲ��������ٽ����������ٽ���
	
//	xTaskCreate((TaskFunction_t)INSTask,  //��Ҫ����������mpu6500�������̬���㣬�ó�ŷ����
//                (const char *)"INSTask",
//                (uint16_t)INS_TASK_SIZE,
//                (void *)NULL,
//                (UBaseType_t)INS_TASK_PRIO,
//                (TaskHandle_t *)&INSTask_Handler);
	
#if 0
	xTaskCreate((TaskFunction_t)REMOTE_Task,         //���ң�������ݴ�������
                (const char *)"REMOTE_Task",         //��������
                (uint16_t)REMOTE_STK_SIZE,           //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)REMOTE_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t *)&RemoteTask_Handler);//������
#endif
			
//	xTaskCreate((TaskFunction_t)GIMBAL_Task,         //�����̨��������
//                (const char *)"GIMBAL_Task",         //��������
//                (uint16_t)GIMBAL_STK_SIZE,           //�����ջ��С
//                (void *)NULL,                        //���ݸ��������Ĳ���
//                (UBaseType_t)GIMBAL_TASK_PRIO,       //�������ȼ�
//                (TaskHandle_t *)&GIMBALTask_Handler);//������
//	
//	xTaskCreate((TaskFunction_t)DETECT_Task,         //һ����ͨ��������
//                (const char *)"DETECT_Task",         //��������
//                (uint16_t)Detect_STK_SIZE,           //�����ջ��С
//                (void *)NULL,                        //���ݸ��������Ĳ��� 
//                (UBaseType_t)Detect_TASK_PRIO,       //�������ȼ�
//                (TaskHandle_t *)&DetectTask_Handler);//������
//								
//	xTaskCreate((TaskFunction_t)CHASSIS_Task,         //��ɵ��̿�������
//                (const char *)"CHASSIS_Task",         //��������
//                (uint16_t)Chassis_STK_SIZE,           //�����ջ��С
//                (void *)NULL,                         //���ݸ��������Ĳ���
//                (UBaseType_t)Chassis_TASK_PRIO,       //�������ȼ�
//                (TaskHandle_t *)&ChassisTask_Handler);//������
	
#if 0	
	xTaskCreate((TaskFunction_t)RefereeTask,          //������
                (const char *)"RefereeTask",          //��������
                (uint16_t)Referee_STK_SIZE,           //�����ջ��С
                (void *)NULL,                         //���ݸ��������Ĳ���
                (UBaseType_t)Referee_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t *)&RefereeTask_Handler);//������							
#endif
	
	xTaskCreate((TaskFunction_t)SAFECHECK_Task,         //������ȫ�������	
                (const char *)"SAFECHECK_Task",         //��������
                (uint16_t)SAFE_STK_SIZE,                //�����ջ��С
                (void *)NULL,                           //���ݸ��������Ĳ���
                (UBaseType_t)SAFE_TASK_PRIO,            //�������ȼ�
                (TaskHandle_t *)&SafecheckTask_Handler);//������

	xTaskCreate((TaskFunction_t)TEST_Task,              //���������������	
                (const char *)"TEST_Task",              //��������
                (uint16_t)TEST_STK_SIZE,                //�����ջ��С
                (void *)NULL,                           //���ݸ��������Ĳ���
                (UBaseType_t)TEST_TASK_PRIO,            //�������ȼ�
                (TaskHandle_t *)&TestTask_Handler);     //������
	
	vTaskDelete(StartTask_Handler);  //ɾ����ʼ����
	taskEXIT_CRITICAL();             //�˳��ٽ���
}



void StartTast()
{
	xTaskCreate((TaskFunction_t)START_Task,          //������
                (const char *)"START_Task",          //��������
                (uint16_t)START_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)START_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&StartTask_Handler); //������
}






