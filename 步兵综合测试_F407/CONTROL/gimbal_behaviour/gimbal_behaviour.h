#ifndef __GIMBAL_BEHAVIOUR_H
#define __GIMBAL_BEHAVIOUR_H
#include "main.h"
#include "Task_Gimbal.h"

//typedef enum
//{
//  GIMBAL_ZERO_FORCE = 0, //��̨����
//  GIMBAL_INIT,           //��̨��ʼ��
//  GIMBAL_CALI,           //��̨У׼
//  GIMBAL_ABSOLUTE_ANGLE, //��̨�����Ǿ��ԽǶȿ���
//  GIMBAL_RELATIVE_ANGLE, //��̨�������ֵ��ԽǶȿ���
//  GIMBAL_MOTIONLESS,     //��̨��ң����������һ��ʱ��󱣳ֲ���������������Ư��
//} gimbal_behaviour_e;


extern void Gimbal_behaviour_mode_set(gimbal_first_control_t *fir_gimbal_behaviour, gimbal_second_control_t *sec_gimbal_behaviour);

#endif
