#ifndef __SPEEDDEF_H
#define __SPEEDDEF_H


/********************底盘********************/ 
/* 底盘电机移动速度设定 */ 
#define M3508_MAX_OUTPUT_CURRENT  5000  //m3508电机最大电流输出  
#define M2006_MAX_OUTPUT_CURRENT  9500   //m2006电机最大电流输出

/* 低通滤波比例 */
#define CHASSIS_FIRST_ORDER_FILTER_K  0.0510f  //0.0110f 越小越平稳，灵敏度越低；越高输出不稳，但灵敏度更高  |  0.26f  |  0.0097f

/* 遥控转换为速度 */
//#define CHASSIS_VX_RC_SEN  4.8f
//#define CHASSIS_VY_RC_SEN  4.8f
//#define CHASSIS_WZ_RC_SEN  3.8f

/* 底盘运动过程最大速度 */
//#define MAX_CHASSIS_SPEED_X  3.7f  //底盘运动过程最大进退速度
//#define MAX_CHASSIS_SPEED_Y  2.7f  //底盘运动过程最大平移速度
//#define MAX_CHASSIS_SPEED_Y  2.7f  //底盘运动过程最大平移速度

/* 电机转速转为移速(m/s) */
//#define M3508_SPEED_RATE  0.000413367447f


#define CHASSIS_ROTATION_SPEED    2000      //小陀螺的旋转速度 
#define CHASSIS_ROTATION_MOVE_SPEED  1700   //小陀螺移动时为防止轨迹失真减转速 
#define CHASSIS_TWIST_SPEED       1600      //扭腰速度

#define CHASSIS_MOTOR_MAX_I    2000.0f   //3508最大电流输入
#define CHASSIS_MOTOR_MAX_OUT  16000.0f  //3508最大电流输出

/*****键盘鼠标遥控速度设置******/ 
#define MOUSE_YAW_SPEED       0.021f    //鼠标yaw轴速度增益
#define MOUSE_PITCH_SPEED     0.08f     //鼠标pitch轴速度增益0.13
#define RC_YAW_SPEED          0.0026f   //遥控器yaw轴速度增益
#define RC_PITCH_SPEED        0.0026f   //遥控器pitch轴速度增益 0.0026

/*************pitch和yaw输出量限制************/
#define YAW_OUTPUT_LIMIT         11000 
#define YAW_INIT_OUTPUT_LIMIT    8000
#define PITCH_OUTPUT_LIMIT       8000 
#define PITCH_INIT_OUTPUT_LIMIT  5000

/*************减速电机启动电流补偿（快速启动）**********/
#define GEAR_MOTOR_START_CURRENT   300
#define GEAR_MOTOR_REDUCE_CURRENT  1.21



#endif
