/**
  ******************************************************************************
  * @file       Task_Remote.c/h
  * @brief      完成遥控器数据处理。
  ******************************************************************************
  */
#include "Task_Remote.h"
#include "RemoteControl.h"
#include "Task_Start.h"
#include "Task_Detect.h"
#include "Task_Chassis.h"
#include "Task_Gimbal.h"
#include "Task_Safecheck.h"


/*****************************操作键位说明书**********************************************************/
/* 一、遥控模式：
         1.底盘跟随  ：左中右上
         2.扭腰模式  ：左下右上
         3.底盘小陀螺：左上右上
         4.打符模式  ：左上右中（底盘以云台坐标轴移动，不跟随）
         5.自瞄模式  ：左下右中
         6.键盘模式  ：左中右中
         7.发弹      ：在初始化完成且右开关打上或中情况下，左上角波轮拉最上发弹，回中停止射弹
         8.关机      ：右下				 
	 二、键鼠模式：
         1.基本运动：WASD
         2.云台运动：鼠标
         3.发射：    鼠标左键单点单发，按住连发
         4.加速（电容放电）：    按住shift（逮虾户）
         5.扭腰模式： F  （按一下进入，再按一下返回）
         6.自瞄模式： 鼠标右键  （按住）
         7.补给模式：  G  （按一下云台往一边转，补完弹再按一下回来）
         8.高射速模式：C （按一次进入）（这个模式没用了，之前全部最大射速打）
         9.低射速模式：V （按一次进入）
         10.退弹模式： Z  （按住）
         11.炮台模式： Ctrl （按住，只能控制云台，底盘不动）
         12.打符模式： X （一次进入）
         13.小陀螺模式：R（按一次进入）
		 
每次切换状态，都先进入待机模式，也就是重新初始化
                                                                                                    */
/****************************************************************************************************/
/*
   部分简写：RC：remote contral   遥控器
             MK: mouse key        键鼠
*/

/*遥控器或键鼠的处理值*/
RC_Deal_data Remote_data;
/*堆栈剩余*/
extern TaskHandle_t RemoteTask_Handler; // 栈大小

static u8 s2_location = 0;     //该变量为记录该次遥控模式状态（s2）
//extern int8_t INITIALIZE_flag; //发送给底盘的初始化成功标志
extern int gimbal_work_mode;   //云台状态切换控制量处理标志位


void REMOTE_Task(void *pvParameters)
{
    //加载时间
    vTaskDelay(REMOTE_TASK_INIT_TIME);

    while (1)
    {
		/* 状态设置为工作 这样就不用进入初始化，直接可以操作，s1、s2坏了也可以用 不用直接注释掉就好了 */
		work_st = WORKING; 
		
        Select_Ctl_Mode(); //遥控数据处理
		
        vTaskDelay(REMOTE_CONTROL_TIME_MS); //任务周期
    }
}

/* 遥控选择模式函数 */
void Select_Ctl_Mode()
{
	//右开关打上，遥控控制
    if (rc_ctrl.rc.s2 == RC_SW_UP) 
    {
		/* 判断状态是否更新 */
        if (s2_location == RC_SW_MID) //上一次状态为s2打中，更新机器人状态
        {
            chassis_control.chassis_mode = CHASSIS_FOLLOW; //底盘默认工作模式为跟随
            Remote_reload();  //摇杆量清零
        }

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
        s2_location = RC_SW_UP; //s2为上
    }
	//右开关打中，键盘控制
    if (rc_ctrl.rc.s2 == RC_SW_MID) 
    {
		work_st = POWEROFF;    //进入待机模式
//        INITIALIZE_flag = 0;   //初始化标志位清零
        s2_location = RC_SW_MID; //s2归下
        Remote_reload();       //摇杆量清零
    }
	//右开关打下，停止工作
    if (rc_ctrl.rc.s2 == RC_SW_DOWN) 
    {
        work_st = POWEROFF;    //进入待机模式
//        INITIALIZE_flag = 0;   //初始化标志位清零
        s2_location = RC_SW_DOWN; //s2归下
        Remote_reload();       //摇杆量清零
    }
    if (rc_ctrl.rc.s2 == 0)
    {
        work_st = POWEROFF;    //进入待机模式
//        INITIALIZE_flag = 0;   //初始化标志位清零
        Remote_reload();       //摇杆量清零
    }
}

/* 遥控器数据处理 */
void RC_Data_Process(void)
{
//	rc_deadline_limit(rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[0], 10);  //死区限制
//	rc_deadline_limit(rc_ctrl.rc.ch[1], rc_ctrl.rc.ch[1], 10);  //死区限制
//	rc_deadline_limit(rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[2], 10);  //死区限制
//	rc_deadline_limit(rc_ctrl.rc.ch[3], rc_ctrl.rc.ch[3], 10);  //死区限制
	
    /*将遥控器值赋给遥控量*/
    if (work_st == WORKING || work_st == AUTOATTACK || work_st == AUTOBUFF) //手动控制
    {
        Remote_data.RC_ch0 = (rc_ctrl.rc.ch[0]);  //ch0
        Remote_data.RC_ch1 = (rc_ctrl.rc.ch[1]);  //ch1
		
        Remote_data.RC_ch2 += (rc_ctrl.rc.ch[2]) * RC_YAW_SPEED;  //Y轴位置环量累加  0.021  RC_YAW_SPEED
        Remote_data.RC_ch2 = loop_fp32_constrain(Remote_data.RC_ch2, -180.0f, 180.0f); //循环限幅
        Remote_data.RC_ch3 += (rc_ctrl.rc.ch[3]) * 0.04f;  //P轴位置环量累加  0.08  RC_PITCH_SPEED
		
        Remote_data.RC_sw = -(rc_ctrl.rc.ch[4]);  //遥控器拨轮值，中值为1024，上下限两万多。处理后中值为0
    }
    else //待机模式，初始化模式，补给模式  均不可手动控制
    {
        Remote_reload(); //摇杆量清零
    }
}



/* 摇杆量清零 */
void Remote_reload(void)
{
    Remote_data.RC_ch0 = 0;
    Remote_data.RC_ch1 = 0;
    Remote_data.RC_ch2 = 0;
    Remote_data.RC_ch3 = 0;
    Remote_data.RC_sw = 0;
	
//	rc_ctrl.rc.ch[0] = 0;
//	rc_ctrl.rc.ch[1] = 0;
//	rc_ctrl.rc.ch[2] = 0;
//	rc_ctrl.rc.ch[3] = 0;
//	rc_ctrl.rc.ch[4] = 0;
//	rc_ctrl.rc.s1 = RC_SW_MID;
//	rc_ctrl.rc.s2 = RC_SW_DOWN;
//	rc_ctrl.mouse.x = 0;
//	rc_ctrl.mouse.y = 0;
//	rc_ctrl.mouse.z = 0;
//	rc_ctrl.mouse.press_l = 0;
//	rc_ctrl.mouse.press_r = 0;
//	rc_ctrl.key.v = 0;
}
