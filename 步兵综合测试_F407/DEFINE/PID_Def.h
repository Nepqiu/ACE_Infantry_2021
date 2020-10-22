#ifndef __PIDDEF_H
#define __PIDDEF_H

#define PI 3.1415926f

//���̵��pid����
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

#define CHASSIS_MOVE_FOLLOW_P    0.2  //�����˶�����PID   0.015    //�������
#define CHASSIS_MOVE_FOLLOW_I    0
#define CHASSIS_MOVE_FOLLOW_D    0.297

#define CHASSIS_ROTATE_FOLLOW_P  9.0  //���̾�ֹ����PID   8.0
#define CHASSIS_ROTATE_FOLLOW_I  0.0   //0.01
#define CHASSIS_ROTATE_FOLLOW_D  0.0   //5.02   10.02


//P�����ǽ��ٶȻ�
#define GIMBAL_P_PITCH_P  38.0f   //Pλ�û�  53   410    160       58
#define GIMBAL_P_PITCH_I  0.0     //8f      
#define GIMBAL_P_PITCH_D  0.0     //0.0f       360       100     100

#define GIMBAL_S_PITCH_P  7.5f   //P�ٶȻ�(��Ҫ��i)       1.5     9.5
#define GIMBAL_S_PITCH_I  0.0f      
#define GIMBAL_S_PITCH_D  0.0f   

//���������
#define GIMBAL_P_YAW_P  130.0f   //Yλ�û�    150     62    130
#define GIMBAL_P_YAW_I  0.0f       
#define GIMBAL_P_YAW_D  100.0f   //           0      100   100

#define GIMBAL_S_YAW_P  9.0f     //Y�ٶȻ�     8      10     9
#define GIMBAL_S_YAW_I  0.0f   
#define GIMBAL_S_YAW_D  0.0f     //2                   0



#endif
