/**
  ******************************************************************************
  * @file       Task_Chassis.c/h
  * @brief      完成底盘控制任务。
  ******************************************************************************
  */
#ifndef __TASK_CHASSIS_H
#define __TASK_CHASSIS_H
#include "main.h"
#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "maths.h"
#include "pid.h"

/*OS控制任务周期以及启动时间*/
#define CHASSIS_TASK_INIT_TIME  5
#define CHASSIS_CONTROL_TIME    2


/*底盘模式*/
typedef enum
{
	CHASSIS_STOP,        //停止
	CHASSIS_INITIALIZE,  //初始化中
	CHASSIS_POWEROFF,    //待机
	CHASSIS_WORKING,     //工作
	
	CHASSIS_FOLLOW,      //跟随
	CHASSIS_NO_FOLLOW,   //不跟随
	
	CHASSIS_TWIST_WAIST, //扭腰
	CHASSIS_ROTATION,    //小陀螺
} Chassis_mode_e;


typedef struct
{
    fp32 speed;
    fp32 speed_set;
	
    int16_t output;
} Motor_t;


typedef struct //底盘整体数据结构体
{
	const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针
	
	Motor_t chassis_motor[4];           //底盘电机数据(包含电机统一结构体指针)
	PidTypeDef chassis_speed_pid[4];    //pid

	Chassis_mode_e chassis_mode;        //底盘控制状态机
	Chassis_mode_e last_chassis_mode;   //底盘上次控制状态机
	
	first_order_filter_type_t LowFilt_chassis_vx;  //低通滤波器
	first_order_filter_type_t LowFilt_chassis_vy;  //低通滤波器

//	fp32 speed_x;                         //底盘速度 前进方向 前为正，单位 m/s
//	fp32 speed_y;                         //底盘速度 左右方向 左为正  单位 m/s
//	fp32 speed_z;                         //底盘旋转角速度，逆时针为正 单位 rad/s
	fp32 speed_x_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
	fp32 speed_y_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
	fp32 speed_z_set;                     //底盘设定旋转角速度，逆时针为正 单位 rad/s
	
	fp32 chassis_gimbal_angel;    //底盘与云台的角度
	
//	fp32 max_speed_x;   //前进方向最大速度 单位m/s
//	fp32 min_speed_x;   //前进方向最小速度 单位m/s
//	fp32 max_speed_y;   //左右方向最大速度 单位m/s
//	fp32 min_speed_y;   //左右方向最小速度 单位m/s
	
//	fp32 chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
//	fp32 chassis_pitch; //陀螺仪和云台电机叠加的pitch角度
//	fp32 chassis_roll;  //陀螺仪和云台电机叠加的roll角度
} chassis_control_t;




//底盘控制数据 static
extern chassis_control_t  chassis_control; 

//底盘移动跟随PID
extern PidTypeDef CHASSIS_MOVE_FOLLOW_PID; 
//底盘旋转跟随PID
extern PidTypeDef CHASSIS_ROTATE_FOLLOW_PID;


//底盘主任务
extern void CHASSIS_Task(void *pvParameters);
//底盘控制量设置	
extern void chassis_set_remote(int16_t ch0, int16_t ch1, int16_t ch2);

#endif
