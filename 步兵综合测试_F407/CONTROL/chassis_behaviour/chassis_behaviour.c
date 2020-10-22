/**
******************************************************************************
* @file       chassis_behaviour.c/h
* @brief      底盘状态机。
******************************************************************************
*/
#include "chassis_behaviour.h"
#include "Task_Chassis.h"
#include "Task_Gimbal.h"
#include "Task_Detect.h"
#include "RemoteControl.h"
#include "chassis_behaviour.h"
#include "gimbal_behaviour.h"
#include "rmmotor.h"
#include "Task_Remote.h"


static void chassis_mode_choose(chassis_control_t *chassis_mode_set);  //底盘模式选择
static void Chassis_Follow(chassis_control_t *Chassis_Follow_control); //底盘跟随
static void Chassis_Twist(chassis_control_t *Chassis_Twist_control);   //底盘扭腰
static void Chassis_Rotation(chassis_control_t *Chassis_Rotation_control);        //底盘小陀螺
static void Chassis_Independent(chassis_control_t *Chassis_Independent_control);  //底盘不跟随


static float Chassis_ch0 = 0.0f, Chassis_ch1 = 0.0f, Chassis_ch2 = 0.0f; //底盘电机受控量



//底盘模式
void chassis_behaviour_mode_set(chassis_control_t *chassis_behaviour)
{
//	chassis_behaviour->chassis_mode = CHASSIS_FOLLOW;
	chassis_behaviour->chassis_mode = CHASSIS_NO_FOLLOW;
//	work_st = WORKING;
	
	/* 底盘模式选择 */
	chassis_mode_choose(chassis_behaviour);
	
	//停止
	if(chassis_behaviour->chassis_mode == CHASSIS_STOP)
	{
		Remote_reload();  //遥控器数值清零
	}
	//跟随
	if(chassis_behaviour->chassis_mode == CHASSIS_FOLLOW)
	{
		Chassis_Follow(chassis_behaviour); //跟随
	}
	//不跟随
	if(chassis_behaviour->chassis_mode == CHASSIS_NO_FOLLOW)
	{
		Chassis_Independent(chassis_behaviour); //底盘不跟随
	}
	//扭腰
	if(chassis_behaviour->chassis_mode == CHASSIS_TWIST_WAIST)
	{
		Chassis_Twist(chassis_behaviour); //扭腰
	}
	//小陀螺
	if(chassis_behaviour->chassis_mode == CHASSIS_ROTATION)
	{
		Chassis_Rotation(chassis_behaviour); //小陀螺
	}
}

/*======底盘模式选择========*/
static void chassis_mode_choose(chassis_control_t *chassis_mode_set)
{
	//右开关打上，遥控控制
    if (rc_ctrl.rc.s2 == RC_SW_UP) 
    {
		/* 判断之前的状态是什么 */
        if (chassis_mode_set->last_chassis_mode == CHASSIS_POWEROFF) //之前的状态为待机
        {
            chassis_mode_set->chassis_mode = CHASSIS_INITIALIZE; //状态设置为初始化
            Remote_reload();      //摇杆量清零
        }
        else /* 初始化状态 手动状态 补给状态 双云台状态 */
        {
			/* 检测云台Yaw轴、Pitch轴、副云台初始化完成没有 */
            if (/*(chassis_mode_set->chassis_mode != CHASSIS_INITIALIZE) && */Gimbal_all_init_detection()) //状态不为初始化
            {
				chassis_mode_set->chassis_mode = CHASSIS_WORKING; //设置为正常模式（可以工作）
            }
			if(chassis_mode_set->chassis_mode == CHASSIS_WORKING)
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
		chassis_mode_set->last_chassis_mode = CHASSIS_STOP; //暂时没有键盘控制
        Remote_reload();       //摇杆量清零
    }
	//右开关打下，停止工作
    if (rc_ctrl.rc.s2 == RC_SW_DOWN) 
    {
		chassis_mode_set->last_chassis_mode = CHASSIS_POWEROFF;  //进入待机状态
        Remote_reload();       //摇杆量清零
    }
//	if (rc_ctrl.rc.s2 == 0)
//	{
//		work_st = POWEROFF;    //进入待机模式
//		Remote_reload();       //摇杆量清零
//	}
}

/*====底盘跟随状态控制====*/
static void Chassis_Follow(chassis_control_t *Chassis_Follow_control)
{
//	Chassis_ch2 += ((Chassis_Follow_control->chassis_RC->rc.ch[2]) * RC_YAW_SPEED);  //Y轴位置环量累加  0.021  RC_YAW_SPEED
//	Chassis_ch2 = loop_fp32_constrain(Chassis_ch2, -180.0f, 180.0f);
	
	Chassis_ch0 = Chassis_Follow_control->chassis_RC->rc.ch[0];
	Chassis_ch1 = Chassis_Follow_control->chassis_RC->rc.ch[1];
	
	//获取云台与底盘的差角
    Chassis_ch2 = chassis_control.chassis_gimbal_angel;
	
	//将跟随底盘的pid目标值设定为差角
    CHASSIS_ROTATE_FOLLOW_PID.SetValue = Chassis_ch2;
	
	//将经过pid计算的输出值传给 Chassis_ch2
    Chassis_ch2 = Location_Pid_Int32(&CHASSIS_ROTATE_FOLLOW_PID, 0);
	
	//底盘跟随，云台差角控制ch2转向
    chassis_set_remote(Chassis_ch0, Chassis_ch1, Chassis_ch2); 
}

/*======底盘不跟随状态控制========*/
static void Chassis_Independent(chassis_control_t *Chassis_Independent_control)
{
//	//*云底分离移动算法：云台为主坐标轴，目标值分解到底盘坐标轴（与小陀螺一样）
//	if (Remote_data.RC_ch1 || Remote_data.RC_ch0)
//	{
//		Chassis_ch1 = cos(Yaw_different_angle / 180 * PI) * Remote_data.RC_ch1; //需要用弧度单位计算
//		Chassis_ch0 = -sin(Yaw_different_angle / 180 * PI) * Remote_data.RC_ch1;
//		if (Remote_data.RC_ch0)
//		{
//			Chassis_ch1 += sin(Yaw_different_angle / 180 * PI) * Remote_data.RC_ch0;
//			Chassis_ch0 += cos(Yaw_different_angle / 180 * PI) * Remote_data.RC_ch0;
//		}
//	}
//	else
//	{
//		Chassis_ch0 = 0;
//		Chassis_ch1 = 0;
//	}
	
	Chassis_ch0 = Chassis_Independent_control->chassis_RC->rc.ch[0];
	Chassis_ch1 = Chassis_Independent_control->chassis_RC->rc.ch[1];

    //*传入值处理
    chassis_set_remote(Chassis_ch0, Chassis_ch1, 0);
}

/*===底盘扭腰状态控制=====*/
static void Chassis_Twist(chassis_control_t *Chassis_Twist_control)
{
	
}

/*=====底盘小陀螺状态控制======*/
static void Chassis_Rotation(chassis_control_t *Chassis_Rotation_control)
{
    /* 小陀螺旋转移动算法：云台为主坐标轴，目标值分解到底盘坐标轴 */
    if (Remote_data.RC_ch1 || Remote_data.RC_ch0) //此时有移动
    {
        Chassis_ch2 = CHASSIS_ROTATION_MOVE_SPEED; //小陀螺移动时减速

        Chassis_ch1 = cos(chassis_control.chassis_gimbal_angel / 180 * PI) * Remote_data.RC_ch1; //需要用弧度单位计算
        Chassis_ch0 = -sin(chassis_control.chassis_gimbal_angel / 180 * PI) * Remote_data.RC_ch1;
        if (Remote_data.RC_ch0)
        {
            Chassis_ch1 += sin(chassis_control.chassis_gimbal_angel / 180 * PI) * Remote_data.RC_ch0;
            Chassis_ch0 += cos(chassis_control.chassis_gimbal_angel / 180 * PI) * Remote_data.RC_ch0;
        }
    }
	/* 原地转快速 */
    else 
    {
        //*开环固定转速
        Chassis_ch2 = CHASSIS_ROTATION_SPEED; //（左正右负）
        Chassis_ch0 = 0;
        Chassis_ch1 = 0;
    }

    //*传入值处理
    chassis_set_remote(Chassis_ch0*2, Chassis_ch1*2, Chassis_ch2);
}




