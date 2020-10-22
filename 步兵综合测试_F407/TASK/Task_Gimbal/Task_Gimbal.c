/**
  ******************************************************************************
  * @file       Task_Gimbal.c/h
  * @brief      完成云台控制任务。
  ******************************************************************************
  */
  
#include "Task_Gimbal.h"
#include "gimbal_behaviour.h"
#include "Task_Remote.h"
#include "maths.h"
#include "rmmotor.h"
#include "filter.h"
#include "CAN_Receive.h"
#include "IMU.h"
#include "photoelectric.h"
#include "Task_Chassis.h"
#include "Task_Detect.h"
#include "RemoteControl.h"
#include "chassis_behaviour.h"


/*--------------------变量-----------------------*/
//申明拨弹电机变量
gimbal_fire_control_t fire_control;
//申明副云台电机变量
gimbal_second_control_t sec_gimbal_control;
//申明主云台变量
gimbal_first_control_t fir_gimbal_control;

//int16_t pitch_real_jscope = 0;        //P轴打印实际曲线
//int16_t pitch_set_jscope = 0;         //P轴打印设定曲线
//float yaw_real_jscope = 0;            //Y轴打印实际曲线
//float yaw_set_jscope = 0;             //Y轴打印设定曲线
//float sec_yaw_set_jscope = 0;         //副Y轴打印设定曲线
//float sec_yaw_real_jscope = 0;        //副Y轴打印实际曲线

//补给状态标志位(0:归中状态，进入补给状态  1:云台90度转动中  2:到达指定位置  3:归中中   4：补给模式结束)
int8_t GIMBAL_SUPPLY_FLAG = 0;
//发送给底盘的初始化成功标志
//int8_t INITIALIZE_flag = 0;

static float Gimbal_set_position = 0;        //主云台pid设定位置

/*--------------------函数-----------------------*/
/* 云台控制 */
static void Gimbal_Task_Control(gimbal_first_control_t *gimbal_task_control, gimbal_second_control_t *sec_gimbal_task_control);
static void Gimbal_remote_mode_choose(gimbal_first_control_t *fir_gimbal_choose, gimbal_second_control_t *sec_gimbal_choose);
static void Gimbal_data_init(gimbal_first_control_t *gimbal_initialization, gimbal_second_control_t *sec_gimbal_initialization);

/* 控制算法 */
static void Gimbal_Angle_Limit(void);      //云台角度限制
static void Correct_Yaw_Zero(void);        //yaw光电校正
static void Correct_Sec_Yaw_Zero(void);    //副yaw光电校正



/* 云台主任务 */
void GIMBAL_Task(void *pvParameters)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
	
	Gimbal_data_init(&fir_gimbal_control, &sec_gimbal_control);

    while (1)
    {
        Gimbal_Task_Control(&fir_gimbal_control, &sec_gimbal_control); //云台工作
		
        vTaskDelay(GIMBAL_CONTROL_TIME_MS);  //系统延时
    }
}


static void Gimbal_data_init(gimbal_first_control_t *gimbal_initialization, gimbal_second_control_t *sec_gimbal_initialization)
{
	BMI160_Zero_Correct();     //发送校准信号给陀螺仪模块
	
#if IMU_BMI160
	yaw_control.last_angle = IMU_t.yaw_angle;
#endif
	
	//开机状态为停止
	gimbal_initialization->fir_gimbal_mode = GIMBAL_STOP; 
	//获取遥控器数据
	gimbal_initialization->gimbal_RC = get_remote_control_point();
}



/**====云台状态控制====**/
static void Gimbal_Task_Control(gimbal_first_control_t *gimbal_task_control, gimbal_second_control_t *sec_gimbal_task_control)
{
	Correct_Yaw_Zero(); //yaw轴码盘修正
//	Correct_Sec_Yaw_Zero();  //副yaw轴码盘修正

    //==============预处理============
	/* 实时监测Yaw校准光电  (到中值返回1) */
	gimbal_task_control->yaw_c.photoelectric_zero = Yaw_Zero_Value();
	/* 实时监测副Yaw校准光电(到中值返回1) */
    sec_gimbal_task_control->photoelectric_zero = Sec_Yaw_Zero_Value();
	
    /* 云台数据发送给底盘 */   //云台初始化成功标志,yaw轴中值到达标志，yaw轴电机输出数据，补给状态标志
	CAN2_gimbal_SetMsg(/*INITIALIZE_flag*/0, gimbal_task_control->yaw_c.output, gimbal_task_control->yaw_c.photoelectric_zero, GIMBAL_SUPPLY_FLAG, sec_gimbal_control.chassis_different_angle);

#if IMU_BMI160
	IMU_Data_Deal(); //外接陀螺仪
#endif
	
#ifdef double_gimbal
	/* 码盘解算出副云台与主云台之间的差角 */
	sec_gimbal_control.Yaw_different_angle = Get_Yaw_Different_Angle(&motor_second_yaw, Sec_YAW_RATIO);
	/* 循环限幅 */
	sec_gimbal_control.chassis_different_angle = loop_fp32_constrain((yaw_control.chassis_different_angle - sec_gimbal_control.chassis_different_angle), -180, 180);
#endif

	//云台模式选择
	Gimbal_remote_mode_choose(gimbal_task_control, sec_gimbal_task_control);

	    
#ifdef board_gimbal
    /* 输出云台电机 和 副云台yaw轴 */
    CAN1_Gimbal_SetMsg(gimbal_task_control->pitch_c.output, 0);  //云台板can1控制P轴和副云台yaw
#endif
	
#ifdef board_chassis
	/* 输出yaw轴 和 供弹2006电机 */
	CAN1_Chassis_Gimbal_Fire(gimbal_task_control->yaw_c.output, 0, 0);  //底盘板can1控制Y轴 | 供弹
#endif
}

/* 云台行为选择 */
static void Gimbal_remote_mode_choose(gimbal_first_control_t *fir_gimbal_choose, gimbal_second_control_t *sec_gimbal_choose)
{
	Gimbal_behaviour_mode_set(fir_gimbal_choose, sec_gimbal_choose);
}



/*===============================云台正常控制========================*/
void Gimbal_Work(gimbal_first_control_t *gimbal_working, int16_t gimbal_ch2, int16_t gimbal_ch3)
{
#if IMU_BMI160
	yaw_control.angle = IMU_t.yaw_angle - yaw_control.last_angle;  //获取云台正常工作时的陀螺仪Y轴角度
#else
	gimbal_working->yaw_c.angle = 0.0f;  // 云台可以随便动，不会回中
#endif
	
	Gimbal_Angle_Limit();  // 云台角度限制
	
    /* 主云台控制 前一个变量为遥控器控制，后一个变量为控制地面绝对角度 */
	Gimbal_set_position = loop_fp32_constrain((-Remote_data.RC_ch2 - (YAW_ANGLE_FLAG * gimbal_working->yaw_c.angle)), -180.0f, 180.0f); //云台设定位置循环限幅
	
    gimbal_working->pitch_c.output = Sliding_Mean_Filter(&gimbal_working->pitch_c.Slidmean_Pitch_Data, gimbal_working->pitch_c.output, 55); //均值滑窗滤波（有滞后）
    gimbal_working->pitch_c.output = first_order_filter(&gimbal_working->pitch_c.LowFilt_Pitch_Data, gimbal_working->pitch_c.output);   //一阶低通滤波

	/* Pitch位置控制 */
    gimbal_working->pitch_c.output = Motor_Position_Speed_Control
	                                 (&gimbal_working->pitch_c.pitch_s_pid, 
	                                 &gimbal_working->pitch_c.pitch_p_pid, 
	                                 0,  /*真实位置*/
	                                 IMU_t.Gyro_X,  /*真实速度*/
	                                 -((Remote_data.RC_ch3) - (motor_pitch.actual_Position * 360 / 1024)),  /*设定位置*/
	                                 PITCH_OUTPUT_LIMIT);  /*输出限制*/  
						   
	/* Yaw位置控制 */
    gimbal_working->yaw_c.output = Motor_Position_Speed_Control
	                               (&gimbal_working->pitch_c.pitch_s_pid, 
	                               &gimbal_working->pitch_c.pitch_p_pid, 
	                               0,  /*真实位置*/
	                               motor_yaw.speed,  /*真实速度*/
	                               Gimbal_set_position,  /*设定位置*/
	                               YAW_OUTPUT_LIMIT);  /*输出限制*/
	
    /*打印曲线*/
//	pitch_real_jscope = pitch_control.output; //jscope观察曲线变化
}



/******************控制算法************************/

/*
* 功能：云台的角度限制
* 输入：遥控值结构体
* 输出：无
* 描述：限制处理后的控制量
*/
static void Gimbal_Angle_Limit(void)
{
    //pitch角度限制
    Remote_data.RC_ch3 = float_limit(Remote_data.RC_ch3, PITCH_ANGLE_LIMIT_UP, PITCH_ANGLE_LIMIT_DOWN);
    //yaw角度限制
//	Remote_data.RC_ch2 = float_limit(Remote_data.RC_ch2, YAW_ANGLE_LIMIT, -YAW_ANGLE_LIMIT);
}

/******************************************/

/*
* 功能：Yaw轴电机码盘位置的光电校正
* 输入：无
* 输出：无
* 描述：云台工作过程中运行次程序会调用检测Y中值光电程序，修正Y电机码盘真实值，中间为0
*/
static int16_t correctYawValue[5] = {0};
static void Correct_Yaw_Zero(void)
{
    u8 i = 0;
    u16 correctSum = 0;

    for (i = 0; i < 5; i++)
    {
        correctYawValue[i - 1] = correctYawValue[i];
    }
    correctYawValue[4] = !fir_gimbal_control.yaw_c.photoelectric_zero; //Yaw_Zero_Value();

    for (i = 0; i < 5; i++)
    {
        correctSum += correctYawValue[i];
    }

    correctSum /= 5;

    if (correctSum == 0)
    {
        motor_yaw.actual_Position = 0;
    }
}

/*
* 功能：副Yaw轴电机码盘位置的光电校正
* 输入：无
* 输出：无
* 描述：云台工作过程中运行次程序会调用检测Y中值光电程序，修正Y电机码盘真实值，中间为0
*/
static int16_t correctSecYawValue[5] = {0};
static void Correct_Sec_Yaw_Zero(void)
{
    u8 i = 0;
    u16 correctSum = 0;

    for (i = 0; i < 5; i++)
    {
        correctSecYawValue[i - 1] = correctSecYawValue[i];
    }
    correctSecYawValue[4] = !sec_gimbal_control.photoelectric_zero; //Yaw_Zero_Value();

    for (i = 0; i < 5; i++)
    {
        correctSum += correctSecYawValue[i];
    }

    correctSum /= 5;

    if (correctSum == 0)
    {
        motor_second_yaw.actual_Position = 0;
    }
}

/*==============================云台初始化检测==============================*/
bool_t Gimbal_all_init_detection(void)
{
    if (fir_gimbal_control.yaw_c.init_flag == 1 && fir_gimbal_control.pitch_c.init_flag == 1 && sec_gimbal_control.init_flag == 1)
    {
		return 1;  //初始化完成
    }
	else
	{
		return 0;  //初始化未完成
	}
}



