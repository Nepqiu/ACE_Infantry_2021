/**
  ******************************************************************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���
  ******************************************************************************
  */

#ifndef __CAN_RECEIVE_H
#define __CAN_RECEIVE_H
#include "main.h"

#define CHASSIS_CAN CAN2
#define GIMBAL_CAN CAN1

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_M3508_MOTOR1_ID = 0x201,
    CAN_M3508_MOTOR2_ID = 0x202,
    CAN_M3508_MOTOR3_ID = 0x203,
    CAN_M3508_MOTOR4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

//rm���ͳһ���ݽṹ��
typedef struct
{
    uint16_t position;
    int16_t  speed;
    int16_t  given_current;
    uint8_t  temperate;
    int16_t  last_position;
	
	int16_t speed_filt;
	int16_t first_Flag;
	int32_t actual_Position;
} motor_measure_t;


/*--------------------����-----------------------*/
//�������̵������
extern motor_measure_t motor_chassis[4];
//���������������
extern motor_measure_t motor_fire;
//����yaw��3508�������
extern motor_measure_t motor_yaw;
//��������̨�������
extern motor_measure_t motor_second_yaw;
//����pitch��������
extern motor_measure_t motor_pitch;



/* CAN1���̰巢�͵����̵�� */
void CAN1_Chassis_SetMsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204);
/* CAN1���̰巢�͵���̨Y����͹������ | Y������205��P������206,���������207 */
void CAN1_Chassis_Gimbal_Fire(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207);
/* CAN1��̨�巢�͵�P���� */
void CAN1_Gimbal_SetMsg(int16_t ESC_201, int16_t ESC_202);

/* CAN2 ���̰�ң�������ݷ��� */
void CAN2_Chassis_RC_SetMsg(void);
/* CAN2 ���̰�������ݷ��� */
void CAN2_Chassis_MK_SetMsg(void);
/* CAN2 ��̨�巢�� */
void CAN2_gimbal_SetMsg(u8 INITIALIZE_flag, int16_t gimbal_output, u8 YAW_ZERO_FLAG, u8 GIMBAL_SUPPLY_FLAG, int16_t Sec_chassis_angle);

#endif
