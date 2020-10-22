#ifndef __PIDDEF_H
#define __PIDDEF_H

#define PI 3.1415926f

//底盘电机pid参数
#define CHASSIS_MOTOR1_PID_Kp    8.0f
#define CHASSIS_MOTOR1_PID_Ki    0.0f
#define CHASSIS_MOTOR1_PID_Kd    0.2f

#define CHASSIS_MOTOR2_PID_Kp    8.0f
#define CHASSIS_MOTOR2_PID_Ki    0.0f
#define CHASSIS_MOTOR2_PID_Kd    0.2f

#define CHASSIS_MOTOR3_PID_Kp    8.0f
#define CHASSIS_MOTOR3_PID_Ki    0.0f
#define CHASSIS_MOTOR3_PID_Kd    0.2f

#define CHASSIS_MOTOR4_PID_Kp    8.0f
#define CHASSIS_MOTOR4_PID_Ki    0.0f
#define CHASSIS_MOTOR4_PID_Kd    0.2f

#define CHASSIS_MOVE_FOLLOW_P    0.2  //底盘运动跟随PID   0.015    //不用这个
#define CHASSIS_MOVE_FOLLOW_I    0
#define CHASSIS_MOVE_FOLLOW_D    0.297

#define CHASSIS_ROTATE_FOLLOW_P  9.0  //底盘静止跟随PID   8.0
#define CHASSIS_ROTATE_FOLLOW_I  0.0   //0.01
#define CHASSIS_ROTATE_FOLLOW_D  0.0   //5.02   10.02


//P陀螺仪角速度环
#define GIMBAL_P_PITCH_P  38.0f   //P位置环  53   410    160       58
#define GIMBAL_P_PITCH_I  0.0     //8f      
#define GIMBAL_P_PITCH_D  0.0     //0.0f       360       100     100

#define GIMBAL_S_PITCH_P  7.5f   //P速度环(不要加i)       1.5     9.5
#define GIMBAL_S_PITCH_I  0.0f      
#define GIMBAL_S_PITCH_D  0.0f   

//外接陀螺仪
#define GIMBAL_P_YAW_P  130.0f   //Y位置环    150     62    130
#define GIMBAL_P_YAW_I  0.0f       
#define GIMBAL_P_YAW_D  100.0f   //           0      100   100

#define GIMBAL_S_YAW_P  9.0f     //Y速度环     8      10     9
#define GIMBAL_S_YAW_I  0.0f   
#define GIMBAL_S_YAW_D  0.0f     //2                   0



#endif
