/**
  ******************************************************************************
  * @file       Task_Start.c/h
  * @brief      开始任务。
  ******************************************************************************
  */
  
#include "Task_Start.h"
#include "Task_Detect.h"
#include "Task_Chassis.h"
#include "Task_Gimbal.h"
#include "Task_Safecheck.h"
#include "Task_Remote.h"
#include "Task_Test.h"


//WorkStatus work_st = POWEROFF;  //默认工作模式为待机模式
WorkStatus work_st = WORKING;


/* 姿态角解析任务 */
//#define INS_TASK_PRIO  5
//#define INS_TASK_SIZE  200
//static TaskHandle_t INSTask_Handler;

/* 云台控制任务 */
#define GIMBAL_TASK_PRIO  6
#define GIMBAL_STK_SIZE   200
TaskHandle_t GIMBALTask_Handler;

/* 底盘控制任务 */
#define Chassis_TASK_PRIO  6
#define Chassis_STK_SIZE   200
TaskHandle_t ChassisTask_Handler;

/* 遥控数据处理程序 */ 
#define REMOTE_TASK_PRIO  9
#define REMOTE_STK_SIZE  150
TaskHandle_t RemoteTask_Handler;

/* 心跳程序 */
#define Detect_TASK_PRIO  10
#define Detect_STK_SIZE   50
static TaskHandle_t DetectTask_Handler;

/* 安全监测程序 */
#define SAFE_TASK_PRIO  4
#define SAFE_STK_SIZE   80
static TaskHandle_t SafecheckTask_Handler;

/* 电机测试程序 */
#define TEST_TASK_PRIO  5
#define TEST_STK_SIZE   200
static TaskHandle_t TestTask_Handler;

/* 开始任务 */
#define START_TASK_PRIO  1
#define START_STK_SIZE   50
static TaskHandle_t StartTask_Handler;



void START_Task(void *pvParameters)
{
	taskENTER_CRITICAL();  //为了保证对PORTA寄存器的访问不被中断，将访问操作放入临界区。进入临界区
	
//	xTaskCreate((TaskFunction_t)INSTask,  //主要利用陀螺仪mpu6500，完成姿态解算，得出欧拉角
//                (const char *)"INSTask",
//                (uint16_t)INS_TASK_SIZE,
//                (void *)NULL,
//                (UBaseType_t)INS_TASK_PRIO,
//                (TaskHandle_t *)&INSTask_Handler);
	
#if 0
	xTaskCreate((TaskFunction_t)REMOTE_Task,         //完成遥控器数据处理任务
                (const char *)"REMOTE_Task",         //任务名称
                (uint16_t)REMOTE_STK_SIZE,           //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)REMOTE_TASK_PRIO,       //任务优先级
                (TaskHandle_t *)&RemoteTask_Handler);//任务句柄
#endif
			
//	xTaskCreate((TaskFunction_t)GIMBAL_Task,         //完成云台控制任务
//                (const char *)"GIMBAL_Task",         //任务名称
//                (uint16_t)GIMBAL_STK_SIZE,           //任务堆栈大小
//                (void *)NULL,                        //传递给任务函数的参数
//                (UBaseType_t)GIMBAL_TASK_PRIO,       //任务优先级
//                (TaskHandle_t *)&GIMBALTask_Handler);//任务句柄
//	
//	xTaskCreate((TaskFunction_t)DETECT_Task,         //一个普通心跳程序
//                (const char *)"DETECT_Task",         //任务名称
//                (uint16_t)Detect_STK_SIZE,           //任务堆栈大小
//                (void *)NULL,                        //传递给任务函数的参数 
//                (UBaseType_t)Detect_TASK_PRIO,       //任务优先级
//                (TaskHandle_t *)&DetectTask_Handler);//任务句柄
//								
//	xTaskCreate((TaskFunction_t)CHASSIS_Task,         //完成底盘控制任务。
//                (const char *)"CHASSIS_Task",         //任务名称
//                (uint16_t)Chassis_STK_SIZE,           //任务堆栈大小
//                (void *)NULL,                         //传递给任务函数的参数
//                (UBaseType_t)Chassis_TASK_PRIO,       //任务优先级
//                (TaskHandle_t *)&ChassisTask_Handler);//任务句柄
	
#if 0	
	xTaskCreate((TaskFunction_t)RefereeTask,          //任务函数
                (const char *)"RefereeTask",          //任务名称
                (uint16_t)Referee_STK_SIZE,           //任务堆栈大小
                (void *)NULL,                         //传递给任务函数的参数
                (UBaseType_t)Referee_TASK_PRIO,       //任务优先级
                (TaskHandle_t *)&RefereeTask_Handler);//任务句柄							
#endif
	
	xTaskCreate((TaskFunction_t)SAFECHECK_Task,         //创建安全检查任务	
                (const char *)"SAFECHECK_Task",         //任务名称
                (uint16_t)SAFE_STK_SIZE,                //任务堆栈大小
                (void *)NULL,                           //传递给任务函数的参数
                (UBaseType_t)SAFE_TASK_PRIO,            //任务优先级
                (TaskHandle_t *)&SafecheckTask_Handler);//任务句柄

	xTaskCreate((TaskFunction_t)TEST_Task,              //创建电机测试任务	
                (const char *)"TEST_Task",              //任务名称
                (uint16_t)TEST_STK_SIZE,                //任务堆栈大小
                (void *)NULL,                           //传递给任务函数的参数
                (UBaseType_t)TEST_TASK_PRIO,            //任务优先级
                (TaskHandle_t *)&TestTask_Handler);     //任务句柄
	
	vTaskDelete(StartTask_Handler);  //删除开始任务
	taskEXIT_CRITICAL();             //退出临界区
}



void StartTast()
{
	xTaskCreate((TaskFunction_t)START_Task,          //任务函数
                (const char *)"START_Task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
}






