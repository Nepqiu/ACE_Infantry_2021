/**
  ******************************************************************************
  * @file       can_receive.c/h
  * @brief      完成can设备数据收发函数，该文件是通过can中断完成接收
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

//rm电机统一数据结构体
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


/*--------------------变量-----------------------*/
//申明底盘电机变量
extern motor_measure_t motor_chassis[4];
//申明拨弹电机变量
extern motor_measure_t motor_fire;
//申明yaw轴3508电机变量
extern motor_measure_t motor_yaw;
//申明副云台电机变量
extern motor_measure_t motor_second_yaw;
//申明pitch轴电机变量
extern motor_measure_t motor_pitch;



/* CAN1底盘板发送到底盘电机 */
void CAN1_Chassis_SetMsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204);
/* CAN1底盘板发送到云台Y电机和供弹电机 | Y轴电机是205，P轴电机是206,供弹电机是207 */
void CAN1_Chassis_Gimbal_Fire(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207);
/* CAN1云台板发送到P轴电机 */
void CAN1_Gimbal_SetMsg(int16_t ESC_201, int16_t ESC_202);

/* CAN2 底盘板遥控器数据发送 */
void CAN2_Chassis_RC_SetMsg(void);
/* CAN2 底盘板键盘数据发送 */
void CAN2_Chassis_MK_SetMsg(void);
/* CAN2 云台板发送 */
void CAN2_gimbal_SetMsg(u8 INITIALIZE_flag, int16_t gimbal_output, u8 YAW_ZERO_FLAG, u8 GIMBAL_SUPPLY_FLAG, int16_t Sec_chassis_angle);

#endif
