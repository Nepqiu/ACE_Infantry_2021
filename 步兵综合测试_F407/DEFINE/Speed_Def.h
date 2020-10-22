#ifndef __SPEEDDEF_H
#define __SPEEDDEF_H


/********************����********************/ 
/* ���̵���ƶ��ٶ��趨 */ 
#define M3508_MAX_OUTPUT_CURRENT  5000  //m3508������������  
#define M2006_MAX_OUTPUT_CURRENT  9500   //m2006������������

/* ��ͨ�˲����� */
#define CHASSIS_FIRST_ORDER_FILTER_K  0.0510f  //0.0110f ԽСԽƽ�ȣ�������Խ�ͣ�Խ��������ȣ��������ȸ���  |  0.26f  |  0.0097f

/* ң��ת��Ϊ�ٶ� */
//#define CHASSIS_VX_RC_SEN  4.8f
//#define CHASSIS_VY_RC_SEN  4.8f
//#define CHASSIS_WZ_RC_SEN  3.8f

/* �����˶���������ٶ� */
//#define MAX_CHASSIS_SPEED_X  3.7f  //�����˶������������ٶ�
//#define MAX_CHASSIS_SPEED_Y  2.7f  //�����˶��������ƽ���ٶ�
//#define MAX_CHASSIS_SPEED_Y  2.7f  //�����˶��������ƽ���ٶ�

/* ���ת��תΪ����(m/s) */
//#define M3508_SPEED_RATE  0.000413367447f


#define CHASSIS_ROTATION_SPEED    2000      //С���ݵ���ת�ٶ� 
#define CHASSIS_ROTATION_MOVE_SPEED  1700   //С�����ƶ�ʱΪ��ֹ�켣ʧ���ת�� 
#define CHASSIS_TWIST_SPEED       1600      //Ť���ٶ�

#define CHASSIS_MOTOR_MAX_I    2000.0f   //3508����������
#define CHASSIS_MOTOR_MAX_OUT  16000.0f  //3508���������

/*****�������ң���ٶ�����******/ 
#define MOUSE_YAW_SPEED       0.021f    //���yaw���ٶ�����
#define MOUSE_PITCH_SPEED     0.08f     //���pitch���ٶ�����0.13
#define RC_YAW_SPEED          0.0026f   //ң����yaw���ٶ�����
#define RC_PITCH_SPEED        0.0026f   //ң����pitch���ٶ����� 0.0026

/*************pitch��yaw���������************/
#define YAW_OUTPUT_LIMIT         11000 
#define YAW_INIT_OUTPUT_LIMIT    8000
#define PITCH_OUTPUT_LIMIT       8000 
#define PITCH_INIT_OUTPUT_LIMIT  5000

/*************���ٵ��������������������������**********/
#define GEAR_MOTOR_START_CURRENT   300
#define GEAR_MOTOR_REDUCE_CURRENT  1.21



#endif
