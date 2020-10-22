/**
  ******************************************************************************
  * @file       Task_Chassis.c/h
  * @brief      完成底盘控制任务。
  ******************************************************************************
  */
#include "Task_Chassis.h"
#include "Task_Detect.h"
#include "RemoteControl.h"
#include "chassis_behaviour.h"
#include "rmmotor.h"


/*--------------------变量-----------------------*/
//底盘控制数据 static
chassis_control_t  chassis_control; 

//底盘移动跟随PID
PidTypeDef CHASSIS_MOVE_FOLLOW_PID; 
//底盘旋转跟随PID
PidTypeDef CHASSIS_ROTATE_FOLLOW_PID; 

#if CHASSIS_TEST_MODE
float lowfilt_ch1_jscope;   //低通滤波vx打印实际曲线
float lowfilt_ch0_jscope;   //低通滤波vy打印实际曲线
#endif

/*--------------------函数-----------------------*/
static void chassis_controlwork(void);
//底盘数据初始化
static void chassis_init(chassis_control_t *chassis_move_init);
//底盘数据更新	
static void chassis_data_update(void);
//底盘遥控模式选择	
static void chassis_remote_mode_choose(void);	
//底盘pid计算
static void chassis_pid_calc(void);	
//底盘功率限制								
//static void chassis_power_limit(void);
#if CHASSIS_TEST_MODE
/* jscope打印曲线 */
static void chassis_jscope_print_curve(void);
#endif



void CHASSIS_Task(void *pvParameters)
{
	//空闲一段时间
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	//底盘数据初始化
	chassis_init(&chassis_control);
	
	while(1)
	{
		//底盘控制
		chassis_controlwork();
		
		//发送遥控器数据给云台板
		CAN2_Chassis_RC_SetMsg();
		
		//can发送电流值
		CAN1_Chassis_SetMsg(chassis_control.chassis_motor[0].output, chassis_control.chassis_motor[1].output, chassis_control.chassis_motor[2].output, chassis_control.chassis_motor[3].output);
		
		vTaskDelay(CHASSIS_CONTROL_TIME);
	}
}



//底盘数据初始化
static void chassis_init(chassis_control_t *chassis_move_init)
{
	//底盘开机状态为停止
	chassis_move_init->chassis_mode = CHASSIS_STOP;
	chassis_move_init->chassis_RC = get_remote_control_point();
	
	//检查遥控器数值是否正确
	RC_data_is_error();
	
	//初始化低通滤波
	first_order_filter_init(&(chassis_move_init->LowFilt_chassis_vx), CHASSIS_FIRST_ORDER_FILTER_K);
	first_order_filter_init(&(chassis_move_init->LowFilt_chassis_vy), CHASSIS_FIRST_ORDER_FILTER_K);
	
	//初次进入更新数据
	chassis_data_update();
}


static void chassis_controlwork(void)
{
	//检查遥控器数值是否正确
	RC_data_is_error();
	
#ifdef board_gimbal
	//计算底盘与云台差角
	yaw_control.chassis_different_angle = Get_Yaw_Different_Angle(&motor_yaw, YAW_RATIO); 
#endif
	
#ifdef board_chassis
//	if ((rc_ctrl.rc.s2 == 3 && rc_ctrl.rc.s1 == 2) || Vision_flag == 1)
//	{
//		sec_gimbal_control.chassis_different_angle = Sec_chassis_angle; //副云台相对底盘的差角
//	}
//	else
//	{
//		yaw_control.chassis_different_angle = Get_Yaw_Different_Angle(&motor_yaw, YAW_RATIO);  //计算底盘与云台差角
//	}
	
	//底盘数据更新
	chassis_data_update();
	//遥控器模式状态设置
	chassis_remote_mode_choose();
	//底盘控制PID计算
	chassis_pid_calc();
	
#ifdef power_limit  //功率限制
	chassis_power_limit(&chassis_control);
#endif

#endif
	
	//计算底盘与云台差角
	chassis_control.chassis_gimbal_angel = Get_Yaw_Different_Angle(&motor_yaw, YAW_RATIO);

}


//底盘数据更新
static void chassis_data_update(void)
{
	u8 i;	
	
	//电机速度更新
	for(i=0;i<4;i++)
	{
		chassis_control.chassis_motor[i].speed = motor_chassis[i].speed;
	}                                                            
}



//底盘遥控模式选择	
static void chassis_remote_mode_choose(void)
{	
	//判断是什么模式
	chassis_behaviour_mode_set(&chassis_control);  
}



//底盘控制量设置
void chassis_set_remote(int16_t ch0, int16_t ch1, int16_t ch2)
{
	//一阶低通滤波计算
	first_order_filter(&(chassis_control.LowFilt_chassis_vx), -ch0);
	first_order_filter(&(chassis_control.LowFilt_chassis_vy), ch1);
	
	/* 打印曲线 */
	lowfilt_ch0_jscope = chassis_control.LowFilt_chassis_vx.out;
	lowfilt_ch1_jscope = chassis_control.LowFilt_chassis_vx.out;
	
	/*输入量根据底盘模式处理*/
    if (work_st == POWEROFF || work_st == INITIALIZE) //关电或初始化阶段
    {
        chassis_control.chassis_motor[0].output = 0;
		chassis_control.chassis_motor[1].output = 0;
		chassis_control.chassis_motor[2].output = 0;
		chassis_control.chassis_motor[3].output = 0;
    }
	else
	{
		//底盘停止 将速度目标值设置为0
		if(chassis_control.chassis_mode == CHASSIS_STOP)
		{
			chassis_control.speed_x_set = 0;
			chassis_control.speed_y_set = 0;
			chassis_control.speed_z_set = 0;
		}
		//底盘 不跟随云台 | 扭腰 | 小陀螺 （现在暂时是无驱动，单单控制x和y方向把z方面去掉了）
		if(chassis_control.chassis_mode == CHASSIS_NO_FOLLOW || chassis_control.chassis_mode == CHASSIS_TWIST_WAIST || chassis_control.chassis_mode == CHASSIS_ROTATION)
		{
			chassis_control.speed_x_set = chassis_control.LowFilt_chassis_vx.out;
			chassis_control.speed_y_set = chassis_control.LowFilt_chassis_vy.out;
			chassis_control.speed_z_set = ch2;
			
			//麦轮运动分解
			chassis_control.chassis_motor[0].speed_set = (-chassis_control.speed_y_set - chassis_control.speed_x_set + chassis_control.speed_z_set) * 5.0f;
			chassis_control.chassis_motor[1].speed_set = ( chassis_control.speed_y_set - chassis_control.speed_x_set + chassis_control.speed_z_set) * 5.0f;
			chassis_control.chassis_motor[2].speed_set = (-chassis_control.speed_y_set + chassis_control.speed_x_set + chassis_control.speed_z_set) * 5.0f;
			chassis_control.chassis_motor[3].speed_set = ( chassis_control.speed_y_set + chassis_control.speed_x_set + chassis_control.speed_z_set) * 5.0f;
		}
		//底盘 跟随云台
		if(chassis_control.chassis_mode == CHASSIS_FOLLOW)
		{
			chassis_control.speed_x_set = chassis_control.LowFilt_chassis_vx.out;
			chassis_control.speed_y_set = chassis_control.LowFilt_chassis_vy.out;
			chassis_control.speed_z_set = ch2;
			
			//麦轮运动分解
			chassis_control.chassis_motor[0].speed_set = (-chassis_control.speed_y_set - chassis_control.speed_x_set + chassis_control.speed_z_set) * 2.0f;
			chassis_control.chassis_motor[1].speed_set = ( chassis_control.speed_y_set - chassis_control.speed_x_set + chassis_control.speed_z_set) * 2.0f;
			chassis_control.chassis_motor[2].speed_set = (-chassis_control.speed_y_set + chassis_control.speed_x_set + chassis_control.speed_z_set) * 2.0f;
			chassis_control.chassis_motor[3].speed_set = ( chassis_control.speed_y_set + chassis_control.speed_x_set + chassis_control.speed_z_set) * 2.0f;

		}
	}
	
#if CHASSIS_TEST_MODE
	chassis_jscope_print_curve();
#endif
}


//底盘控制PID计算
static void chassis_pid_calc(void)
{
//	chassis_control.chassis_motor[0].output = PID_Calc(&(chassis_control.chassis_speed_pid[0]), chassis_control.chassis_motor[0].speed, chassis_control.chassis_motor[0].speed_set);
//	chassis_control.chassis_motor[1].output = PID_Calc(&(chassis_control.chassis_speed_pid[1]), chassis_control.chassis_motor[1].speed, chassis_control.chassis_motor[1].speed_set);
//	chassis_control.chassis_motor[2].output = PID_Calc(&(chassis_control.chassis_speed_pid[2]), chassis_control.chassis_motor[2].speed, chassis_control.chassis_motor[2].speed_set);
//	chassis_control.chassis_motor[3].output = PID_Calc(&(chassis_control.chassis_speed_pid[3]), chassis_control.chassis_motor[3].speed, chassis_control.chassis_motor[3].speed_set);
	
	/*电机PID速度闭环处理*/
	chassis_control.chassis_motor[0].output = Rmmotor_Speed_control(&(chassis_control.chassis_speed_pid[0]), chassis_control.chassis_motor[0].speed_set, chassis_control.chassis_motor[0].speed, M3508_MAX_OUTPUT_CURRENT);
	chassis_control.chassis_motor[1].output = Rmmotor_Speed_control(&(chassis_control.chassis_speed_pid[1]), chassis_control.chassis_motor[1].speed_set, chassis_control.chassis_motor[1].speed, M3508_MAX_OUTPUT_CURRENT);
	chassis_control.chassis_motor[2].output = Rmmotor_Speed_control(&(chassis_control.chassis_speed_pid[2]), chassis_control.chassis_motor[2].speed_set, chassis_control.chassis_motor[2].speed, M3508_MAX_OUTPUT_CURRENT);
	chassis_control.chassis_motor[3].output = Rmmotor_Speed_control(&(chassis_control.chassis_speed_pid[3]), chassis_control.chassis_motor[3].speed_set, chassis_control.chassis_motor[3].speed, M3508_MAX_OUTPUT_CURRENT);                                          

}



#ifdef power_limit
//底盘功率限制	
static void chassis_power_limit(chassis_control_t *chassis_power_limit)
{

}
#endif



#if CHASSIS_TEST_MODE
/* jscope打印曲线 */
static void chassis_jscope_print_curve(void)
{
	
	lowfilt_ch0_jscope = chassis_control.LowFilt_chassis_vx.out;
	lowfilt_ch1_jscope = chassis_control.LowFilt_chassis_vx.out;
}
#endif











