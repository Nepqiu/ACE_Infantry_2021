#include "gimbal_behaviour.h"
#include "Task_Gimbal.h"
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

//云台初始化
static void Gimbal_Location_Init(gimbal_first_control_t *fir_gimbal_location_init, gimbal_second_control_t *sec_gimbal_location_init);             
//云台模式选择
static void gimbal_mode_choose(gimbal_first_control_t *fir_gimbal_mode_choose, gimbal_second_control_t *sec_gimbal_mode_choose);
//Yaw轴初始化
static void Yaw_Init(gimbal_first_control_t *yaw_location_init);
//副Yaw轴初始化
static void Sec_Yaw_Init(gimbal_second_control_t *sec_yaw_location_init);           
//Pitch轴初始化
static void Pitch_Init(gimbal_first_control_t *pitch_location_init);              
//云台无力
static void Gimbal_Stop(gimbal_first_control_t *fir_gimbal_stop, gimbal_second_control_t *sec_gimbal_stop);             


static float Gimbal_ch2 = 0.0f, Gimbal_ch3 = 0.0f; //云台电机受控量


void Gimbal_behaviour_mode_set(gimbal_first_control_t *fir_gimbal_behaviour, gimbal_second_control_t *sec_gimbal_behaviour)
{
	/* 模式选择 */
	gimbal_mode_choose(fir_gimbal_behaviour, sec_gimbal_behaviour);
	
	/* 状态切换 */
    if (Gimbal_all_init_detection() == 0)
    {
        Gimbal_Location_Init(fir_gimbal_behaviour, sec_gimbal_behaviour); //云台初始化
    }
	/* 若为手动模式working */
    if ((fir_gimbal_behaviour->fir_gimbal_mode == GIMBAL_WORKING) && (sec_gimbal_behaviour->sec_gimbal_mode == GIMBAL_WORKING))
    {
		Gimbal_ch2 += (rc_ctrl.rc.ch[2]) * RC_YAW_SPEED;  //Y轴位置环量累加  0.021  RC_YAW_SPEED
        Gimbal_ch2 = loop_fp32_constrain(Gimbal_ch2, -180.0f, 180.0f); //循环限幅
        Gimbal_ch3 += (rc_ctrl.rc.ch[3]) * 0.04f;  //P轴位置环量累加  0.08  RC_PITCH_SPEED
		
        Gimbal_Work(fir_gimbal_behaviour, Gimbal_ch2, Gimbal_ch3);
    }
    else
    {
        Gimbal_Stop(fir_gimbal_behaviour, sec_gimbal_behaviour); //停止
    }
}

static void gimbal_mode_choose(gimbal_first_control_t *fir_gimbal_mode_choose, gimbal_second_control_t *sec_gimbal_mode_choose)
{
	//右开关打上，遥控控制
    if (rc_ctrl.rc.s2 == RC_SW_UP) 
    {
//		/* 判断状态是否更新 */
//		if (s2_location == RC_SW_MID) //上一次状态为s2打中，更新机器人状态
//		{
//			chassis_control.chassis_mode = CHASSIS_FOLLOW; //底盘默认工作模式为跟随
//			Remote_reload();  //摇杆量清零
//		}

		/* 判断之前的状态为 什么 */
        if (work_st == POWEROFF) //之前的状态为待机
        {
            work_st = INITIALIZE; //状态设置为初始化
            Remote_reload();      //摇杆量清零
        }
        else if (work_st == AUTOATTACK || work_st == AUTOBUFF) //之前状态为自动打击或打符模式
        {
            work_st = WORKING; //状态设置为工作
        }
        else /* 初始化状态 手动状态 补给状态 双云台状态 */
        {
            if (work_st != INITIALIZE) //状态不为初始化
            {
                RC_Data_Process(); //遥控器数据处理

                /*开关切换模式*/
                if (rc_ctrl.rc.s1 == RC_SW_UP) //s1（左上）打到上
                {
                    chassis_control.chassis_mode = CHASSIS_ROTATION; //底盘小陀螺模式   CHASSIS_ROTATION
                }
                if (rc_ctrl.rc.s1 == RC_SW_MID) //s1（左上）打到中间
                {
                    chassis_control.chassis_mode = CHASSIS_FOLLOW; //底盘跟随模式  平常遥控状态  CHASSIS_FOLLOW
                }
                if (rc_ctrl.rc.s1 == RC_SW_DOWN) //s1（左上）打到最下
                {
                    chassis_control.chassis_mode = CHASSIS_TWIST_WAIST; //底盘扭腰模式   CHASSIS_TWIST_WAIST
                }
            }
        }
    }
	//右开关打中，键盘控制
    if (rc_ctrl.rc.s2 == RC_SW_MID) 
    {
		work_st = POWEROFF;    //进入待机模式
//		INITIALIZE_flag = 0;   //初始化标志位清零
        Remote_reload();       //摇杆量清零
    }
	//右开关打下，停止工作
    if (rc_ctrl.rc.s2 == RC_SW_DOWN) 
    {
        work_st = POWEROFF;    //进入待机模式
//		INITIALIZE_flag = 0;   //初始化标志位清零
        Remote_reload();       //摇杆量清零
    }
//	if (rc_ctrl.rc.s2 == 0)
//	{
//		work_st = POWEROFF;    //进入待机模式
//		INITIALIZE_flag = 0;   //初始化标志位清零
//		Remote_reload();       //摇杆量清零
//	}
}


/*==============================云台初始化==============================*/
//云台初始程序
static void Gimbal_Location_Init(gimbal_first_control_t *fir_gimbal_location_init, gimbal_second_control_t *sec_gimbal_location_init)
{
	fir_gimbal_location_init->yaw_c.init_flag = 1;
	fir_gimbal_location_init->pitch_c.init_flag = 1;
	sec_gimbal_location_init->init_flag = 1;
	
//	set_gimbal_initflag(yaw_control, pitch_control, sec_gimbal_control, 1);

//	Yaw_Init();     //Y轴初始化
//	Pitch_Init();   //P轴初始化
//	Sec_Yaw_Init(); //副Y轴初始化
	
	Sliding_Mean_Filter_Init(&fir_gimbal_location_init->pitch_c.Slidmean_Pitch_Data); //初始化P轴滑动滤波器

    if (fir_gimbal_location_init->yaw_c.init_flag == 1 && fir_gimbal_location_init->pitch_c.init_flag == 1 && sec_gimbal_location_init->init_flag == 1)
    {
        fir_gimbal_location_init->yaw_c.init_flag = 0;
		fir_gimbal_location_init->pitch_c.init_flag = 0;
		sec_gimbal_control.init_flag = 0;
//        INITIALIZE_flag = 1;
        work_st = WORKING; //开始运行
    }
}

//Yaw轴初始化
static void Yaw_Init(gimbal_first_control_t *yaw_location_init)
{
    if (yaw_location_init->yaw_c.init_flag == 0) //Y轴初始化未成功
    {
		//速度环给900目标值
        yaw_location_init->yaw_c.output = Rmmotor_Speed_control(&yaw_location_init->yaw_c.yaw_s_pid, 900, motor_yaw.speed, 1500);
        
		//码盘接近中值
		if (yaw_location_init->yaw_c.photoelectric_zero == 1)
        {
            yaw_location_init->yaw_c.init_flag = 1;
        }
    }
    else
    {
        yaw_location_init->yaw_c.output = 0;
    }
}

//Pitch轴初始化
static int16_t pitch_init_error = 0; //P轴初始化累失败次数
static int16_t pitch_init_count = 0; //P轴初始化云台归中消抖计数
static void Pitch_Init(gimbal_first_control_t *pitch_location_init)
{
	if (pitch_location_init->pitch_c.init_flag == 0) //P轴未初始化完成
    {
		//位置环目标值为0
		pitch_location_init->pitch_c.output = Motor_Position_Speed_Control(&pitch_location_init->pitch_c.pitch_s_pid, &pitch_location_init->pitch_c.pitch_p_pid, 0, motor_pitch.speed, (motor_pitch.actual_Position * 360 / 1024), PITCH_OUTPUT_LIMIT);                                       

		if (abs(Pitch_Middle_Angle - motor_pitch.position) < 20) //码盘接近中值
		{
			if (pitch_init_count >= 50) //初始化消抖
			{
				pitch_location_init->pitch_c.init_flag = 1;
				pitch_init_count = 0; //复位
				pitch_init_error = 0;
			}
			else
			{
				pitch_init_count++; //成功次数累加
			}
		}
		else
		{
			pitch_init_error++; //初始化失败记录
		}

        if (pitch_init_error == 1500) //多次初始化失败，强制初始化成功
        {
            pitch_location_init->pitch_c.init_flag = 1;
            pitch_init_count = 0; //复位
            pitch_init_error = 0;
        }
    }
    else //P轴初始化完成
    {
        pitch_location_init->pitch_c.output = Motor_Position_Speed_Control
		                                      (&pitch_location_init->pitch_c.pitch_s_pid, 
		                                      &pitch_location_init->pitch_c.pitch_p_pid, 
		                                      0, 
		                                      motor_pitch.speed, 
		                                      (motor_pitch.actual_Position * 360 / 1024), 
		                                      PITCH_OUTPUT_LIMIT);
    }
}

//副云台yaw轴初始化
static void Sec_Yaw_Init(gimbal_second_control_t *sec_yaw_location_init)
{
	
}



/*==================================云台无力==================================*/
static void Gimbal_Stop(gimbal_first_control_t *fir_gimbal_stop, gimbal_second_control_t *sec_gimbal_stop)
{
    fir_gimbal_stop->pitch_c.output = 0;
    fir_gimbal_stop->yaw_c.output = 0;
    sec_gimbal_control.output = 0;
}




