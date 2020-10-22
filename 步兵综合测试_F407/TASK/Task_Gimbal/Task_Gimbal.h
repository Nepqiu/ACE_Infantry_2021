/**
  ******************************************************************************
  * @file       Task_Gimbal.c/h
  * @brief      完成云台控制任务。
  ******************************************************************************
  */

#ifndef __TASK_GIMBAL_H
#define __TASK_GIMBAL_H
#include "main.h"  
#include "pid.h"
#include "RemoteControl.h"


/*OS控制任务周期以及启动时间*/
#define GIMBAL_TASK_INIT_TIME 	1
#define GIMBAL_CONTROL_TIME_MS  2


/* 云台模式 */
typedef enum
{
	GIMBAL_STOP,         //停止
	
	GIMBAL_INITIALIZE,   //初始化状态
	GIMBAL_WORKING,      //手动状态
	
	GIMBAL_AUTOATTACK,   //自瞄状态
	GIMBAL_AUTOBUFF,     //打符状态
//	GIMBAL_REPLENISHMEN, //补给状态
	
	GIMBAL_DOUBLE_GIMBAL,//双云台状态（主云台自瞄，副云台操作） 
	
	GIMBAL_POWEROFF,     //待机状态
} Gimbal_mode_e;


typedef struct  //申明拨弹电机变量
{
    PidTypeDef fire_pid;  //pid
	
	int8_t init_flag;    //初始化成功标志
	
	int16_t output;
	
} gimbal_fire_control_t;

typedef struct  //申明副云台电机变量
{
	Gimbal_mode_e sec_gimbal_mode;
	
    PidTypeDef second_yaw_pid;  //pid

	int8_t init_flag;           //初始化成功标志
	int8_t photoelectric_zero;  //副Y轴中值光电标志
	
	int16_t Yaw_different_angle;       //副云台相对主云台的差角
	int16_t chassis_different_angle;   //副云台相对底盘的差角
	
	int16_t output;
	
} gimbal_second_control_t;

typedef struct  //申明pitch轴电机变量
{
//	Gimbal_mode_e gimbal_pitch_mode;
	
    PidTypeDef pitch_p_pid;  //pid
	PidTypeDef pitch_s_pid;  //pid
	
	int8_t init_flag;    //初始化成功标志
	
	first_order_filter_type_t LowFilt_Pitch_Data;    //P轴低通滤波器
	sliding_mean_filter_type_t Slidmean_Pitch_Data;  //P轴滑动滤波器

	int16_t filt_output; //P轴滤波值
	
	int16_t output;
	
} gimbal_pitch_control_t;

typedef struct
{	
//	Gimbal_mode_e gimbal_yaw_mode;
	
	PidTypeDef yaw_p_pid;  //pid
	PidTypeDef yaw_s_pid;  //pid
	
	bool_t init_flag;           //Y轴初始化成功标志
	int8_t photoelectric_zero;  //Y轴中值光电标志
	
//	int16_t chassis_different_angle;  //云台底盘差角
	
	float angle;              //云台当前角度
	float last_angle;         //云台保存角度

	int16_t filt_output;  //Y轴滤波值
	
	int16_t output;
	
} gimbal_yaw_control_t;


typedef struct
{
	const RC_ctrl_t *gimbal_RC; //底盘使用的遥控器指针
	
	gimbal_pitch_control_t pitch_c;
	gimbal_yaw_control_t yaw_c;
	
	Gimbal_mode_e fir_gimbal_mode;
	
	bool_t gimbal_all_flag;
	
} gimbal_first_control_t;



//申明拨弹电机变量
extern gimbal_fire_control_t fire_control;
//申明副云台电机变量
extern gimbal_second_control_t sec_gimbal_control;
//申明主云台变量
extern gimbal_first_control_t fir_gimbal_control;

//云台初始化成功标志
//extern int8_t INITIALIZE_flag;


/* 云台主任务 */
extern void GIMBAL_Task(void *pvParameters);
/* 云台行为 */
extern void Gimbal_Work(gimbal_first_control_t *gimbal_working, int16_t gimbal_ch2, int16_t gimbal_ch3);
extern bool_t Gimbal_all_init_detection(void);

#endif
