/**
  ******************************************************************************
  * @file       can_receive.c/h
  * @brief      完成can设备数据收发函数，该文件是通过can中断完成接收
  ******************************************************************************
  */

#include "CAN_Receive.h"
#include "RemoteControl.h"
#include "rmmotor.h"
#include "Task_Gimbal.h"
#include "Task_Chassis.h"

//电机数据读取
#define get_motor_M3508(ptr, rx_message)                                                           \
        {                                                                                          \
            (ptr)->position = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);      \
            (ptr)->speed = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);         \
        }

		
/*
   部分简写：RC：remote contral   遥控器
             MK: mouse key        键鼠
*/


/*--------------------变量-----------------------*/
//申明底盘电机变量
motor_measure_t motor_chassis[4];
//申明拨弹电机变量
motor_measure_t motor_fire;
//申明yaw轴3508电机变量
motor_measure_t motor_yaw;
//申明副云台电机变量
motor_measure_t motor_second_yaw;
//申明pitch轴电机变量
motor_measure_t motor_pitch;


extern int8_t GIMBAL_SUPPLY_FLAG; //云台补给标志位

/*--------------------函数-----------------------*/
//can接收处理函数
static void CAN1_gimbal_receive(CanRxMsg *rx_message);
static void CAN1_chassis_receive(CanRxMsg *rx_message);
static void CAN2_gimbal_receive(CanRxMsg *rx_message);
static void CAN2_chassis_receive(CanRxMsg *rx_message);
	
	

//can1接收中断
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx1_message;
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);

#ifdef board_gimbal
		CAN1_gimbal_receive(&rx1_message);
#endif

#ifdef board_chassis
		CAN1_chassis_receive(&rx1_message);
#endif
    }
}

//can2接收中断
void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
		
#ifdef board_gimbal
		CAN2_gimbal_receive(&rx2_message);
#endif

#ifdef board_chassis
		CAN2_chassis_receive(&rx2_message);
#endif
    }
}


/**********************************************************************************/
/***************************************can1***************************************/
/**********************************************************************************/
/* CAN1底盘板发送到底盘电机 */
void CAN1_Chassis_SetMsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //定义一个发送信息的结构体

    CAN1_TxMessage.StdId = CAN_CHASSIS_ALL_ID; //根据820r设置标识符
    CAN1_TxMessage.IDE = CAN_ID_STD;           //指定将要传输的消息的标识符的类型
    CAN1_TxMessage.RTR = CAN_RTR_DATA;         //指定的帧将被传输的消息的类型   数据帧或远程帧
    CAN1_TxMessage.DLC = 8;                    // 发送两帧信息
    CAN1_TxMessage.Data[0] = (unsigned char)(ESC_201 >> 8);
    CAN1_TxMessage.Data[1] = (unsigned char)(ESC_201);
    CAN1_TxMessage.Data[2] = (unsigned char)(ESC_202 >> 8);
    CAN1_TxMessage.Data[3] = (unsigned char)(ESC_202);
    CAN1_TxMessage.Data[4] = (unsigned char)(ESC_203 >> 8);
    CAN1_TxMessage.Data[5] = (unsigned char)(ESC_203);
    CAN1_TxMessage.Data[6] = (unsigned char)(ESC_204 >> 8);
    CAN1_TxMessage.Data[7] = (unsigned char)(ESC_204);

    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //发送信息
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //等待发送结束
        i++;
}

/* CAN1底盘板发送到云台Y电机和供弹电机 | Y轴电机是205，P轴电机是206，供弹电机是207 */
void CAN1_Chassis_Gimbal_Fire(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207) 
{
	u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //定义一个发送信息的结构体

    CAN1_TxMessage.StdId = 0x1ff;      //根据820r设置标识符
    CAN1_TxMessage.IDE = CAN_ID_STD;   //指定将要传输的消息的标识符的类型
    CAN1_TxMessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
    CAN1_TxMessage.DLC = 8;            //指定数据的长度
    CAN1_TxMessage.Data[0] = (unsigned char)(ESC_205 >> 8);
    CAN1_TxMessage.Data[1] = (unsigned char)(ESC_205);
		CAN1_TxMessage.Data[2] = (unsigned char)(ESC_206 >> 8);
		CAN1_TxMessage.Data[3] = (unsigned char)(ESC_206);
    CAN1_TxMessage.Data[4] = (unsigned char)(ESC_207 >> 8);
    CAN1_TxMessage.Data[5] = (unsigned char)(ESC_207);
    CAN1_TxMessage.Data[6] = 0;
    CAN1_TxMessage.Data[7] = 0;
    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //发送信息
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++; //等待发送结束
}

/* CAN1云台板发送到P轴电机 | P轴电机是201，副云台yaw2006是202 */
void CAN1_Gimbal_SetMsg(int16_t ESC_201, int16_t ESC_202)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //定义一个发送信息的结构体

    CAN1_TxMessage.StdId = 0x200;      //根据820r设置标识符
    CAN1_TxMessage.IDE = CAN_ID_STD;   //指定将要传输的消息的标识符的类型
    CAN1_TxMessage.RTR = CAN_RTR_DATA; //指定的帧将被传输的消息的类型   数据帧或远程帧
    CAN1_TxMessage.DLC = 8;            //指定数据的长度
    CAN1_TxMessage.Data[0] = (unsigned char)(ESC_201 >> 8);
    CAN1_TxMessage.Data[1] = (unsigned char)(ESC_201);
    CAN1_TxMessage.Data[2] = (unsigned char)(ESC_202 >> 8);
    CAN1_TxMessage.Data[3] = (unsigned char)(ESC_202);
    CAN1_TxMessage.Data[4] = 0;
    CAN1_TxMessage.Data[5] = 0;
    CAN1_TxMessage.Data[6] = 0;
    CAN1_TxMessage.Data[7] = 0;

    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //发送信息
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++; //等待发送结束
}

/* CAN1底盘板接收处理函数
   底盘板：底盘4电机数据反馈，yaw轴3508数据反馈，拨弹电机2006数据反馈，*/
static void CAN1_chassis_receive(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
		/*底盘电机*/
		case 0x201:
		{
			get_motor_M3508(&motor_chassis[0], rx_message);
//			motor_chassis[0].position = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
//			motor_chassis[0].speed = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
			break;
		}
		case 0x202:
		{
			get_motor_M3508(&motor_chassis[1], rx_message);
//			motor_chassis[1].position = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
//			motor_chassis[1].speed = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
			break;
		}
		case 0x203:
		{
			get_motor_M3508(&motor_chassis[2], rx_message);
//			motor_chassis[2].position = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
//			motor_chassis[2].speed = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
			break;
		}
		case 0x204:
		{
			get_motor_M3508(&motor_chassis[3], rx_message);
//			motor_chassis[3].position = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
//			motor_chassis[3].speed = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
			break;
		}

		/*yaw轴3508电机速度*/
		case 0x205:
		{
//			get_motor_M3508(&motor_yaw, rx_message);
			motor_yaw.position = ((rx_message)->Data[0] << 8) | (rx_message)->Data[1];
			Motor_Actual_Position(&motor_yaw, YAW_RATIO, 8192); //计算yaw电机的真实码盘值
			motor_yaw.speed = ((rx_message)->Data[2] << 8) | (rx_message)->Data[3];
			break;
		}
		
#ifdef fire_work
		/*拨弹电机*/
		case 0x207:
		{
			get_motor_M3508(&motor_fire, rx_message);
//			motor_fire.position = ((rx_message)->Data[0] << 8) | (rx_message)->Data[1];
//			motor_fire.speed = ((rx_message)->Data[2] << 8) | (rx_message)->Data[3];
			break;
		}
#endif
		
		default:
			break;
    }
}

/* CAN1云台板接收处理函数
   云台板：P轴电机速度值反馈，P轴单圈编码器位置反馈，副云台yaw轴2006电机数据反馈*/
static void CAN1_gimbal_receive(CanRxMsg *rx_message)
{
	switch (rx_message->StdId)
    {
		/*Pitch轴编码器位置反馈*/
		case 0x01:
		{
			motor_pitch.position = (rx_message->Data[6] << 24 | rx_message->Data[5] << 16 | rx_message->Data[4] << 8 | rx_message->Data[3]);
			motor_pitch.actual_Position = motor_pitch.position - Pitch_Middle_Angle;   //实际值减去中间值
			break;
		}

		/*P轴3510电机速度反馈*/
		case 0x201:
		{
			motor_pitch.speed = ((rx_message->Data[2] << 8) | rx_message->Data[3]);
			break;
		}

#ifdef double_gimbal
		/*副云台yaw轴2006电机数据反馈*/
		case 0x202:
		{
			motor_second_yaw.position = (rx_message->Data[0] << 8) | rx_message->Data[1];
			Motor_Actual_Position(&motor_second_yaw, Sec_YAW_RATIO, 8192);
			motor_second_yaw.speed = (rx_message->Data[2] << 8) | rx_message->Data[3];
			break;
		}
#endif
	}
}

/**********************************************************************************/
/***************************************can2***************************************/
/**********************************************************************************/
/* CAN2 底盘板遥控器数据发送 */
void CAN2_Chassis_RC_SetMsg(void)
{
    u8 mbox;
    u16 i = 0;

    CanTxMsg CAN2_TxMessage; //定义一个发送信息的结构体

	CAN2_TxMessage.StdId = 0x400;
    CAN2_TxMessage.IDE = CAN_ID_STD;
    CAN2_TxMessage.RTR = CAN_RTR_DATA;
    CAN2_TxMessage.DLC = 8;
	
    CAN2_TxMessage.Data[0] = (unsigned char)(rc_ctrl.rc.ch[2] >> 8);
    CAN2_TxMessage.Data[1] = (unsigned char)(rc_ctrl.rc.ch[2]);
    CAN2_TxMessage.Data[2] = (unsigned char)(rc_ctrl.rc.ch[3] >> 8);
    CAN2_TxMessage.Data[3] = (unsigned char)(rc_ctrl.rc.ch[3]);
    CAN2_TxMessage.Data[4] = (unsigned char)(rc_ctrl.rc.ch[4] >> 8);
    CAN2_TxMessage.Data[5] = (unsigned char)(rc_ctrl.rc.ch[4]);
    CAN2_TxMessage.Data[6] = (unsigned char)((rc_ctrl.rc.s1) * 10 + (rc_ctrl.rc.s2));
    CAN2_TxMessage.Data[7] = (unsigned char)((rc_ctrl.mouse.press_l + 1) * 10 + rc_ctrl.mouse.press_r);

    mbox = CAN_Transmit(CAN2, &CAN2_TxMessage);
    i = 0;
    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //等待发送结束
        i++;
}

/* CAN2 底盘板键盘数据发送 */
void CAN2_Chassis_MK_SetMsg(void)
{
	u16 i = 0;
	u8 mbox;
    CanTxMsg TxMessage;

    TxMessage.StdId = 0x401;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
	
    TxMessage.Data[0] = (unsigned char)(rc_ctrl.mouse.x >> 8);
    TxMessage.Data[1] = (unsigned char)(rc_ctrl.mouse.x);
    TxMessage.Data[2] = (unsigned char)(rc_ctrl.mouse.y >> 8);
    TxMessage.Data[3] = (unsigned char)(rc_ctrl.mouse.y);
    TxMessage.Data[4] = (unsigned char)(rc_ctrl.key.v >> 8);
    TxMessage.Data[5] = (unsigned char)(rc_ctrl.key.v);
    TxMessage.Data[6] = (unsigned char)((rc_ctrl.rc.s1) * 10 + (rc_ctrl.rc.s2));
    TxMessage.Data[7] = (unsigned char)((rc_ctrl.mouse.press_l + 1) * 10 + rc_ctrl.mouse.press_r);
    
	mbox = CAN_Transmit(CAN2, &TxMessage);
    i = 0;
    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++;
}

/* 底盘can2发送遥控值2到云台板  Y轴电机数据 */
void CAN2_Send_Data2_To_Gimbal(void)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg TxMessage;

    TxMessage.StdId = 0x402;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 8;
    TxMessage.Data[0] = (unsigned char)(motor_yaw.actual_Position >> 24);
    TxMessage.Data[1] = (unsigned char)(motor_yaw.actual_Position >> 16);
    TxMessage.Data[2] = (unsigned char)(motor_yaw.actual_Position >> 8);
    TxMessage.Data[3] = (unsigned char)(motor_yaw.actual_Position);
    TxMessage.Data[4] = (unsigned char)(motor_yaw.speed >> 8);
    TxMessage.Data[5] = (unsigned char)(motor_yaw.speed);
    TxMessage.Data[6] = 0;//REFEREE.RobotStatus.shooter_heat0_speed_limit; //射速限制大小
    TxMessage.Data[7] = 0;                                             //热量

    mbox = CAN_Transmit(CAN2, &TxMessage);
    i = 0;
    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++;
}

/* CAN2 云台板发送 */
void CAN2_gimbal_SetMsg(u8 INITIALIZE_flag, int16_t gimbal_output, u8 YAW_ZERO_FLAG, u8 GIMBAL_SUPPLY_FLAG, int16_t Sec_chassis_angle)
{
	u8 mbox;
    u16 i = 0;
	
    CanTxMsg TxMessage;

    TxMessage.StdId = 0x300;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 5;

    TxMessage.Data[0] = (INITIALIZE_flag)*100 + YAW_ZERO_FLAG * 10 + GIMBAL_SUPPLY_FLAG;
    TxMessage.Data[1] = (unsigned char)(gimbal_output >> 8);
    TxMessage.Data[2] = (unsigned char)(gimbal_output);
    TxMessage.Data[3] = (unsigned char)(Sec_chassis_angle >> 8);
    TxMessage.Data[4] = (unsigned char)(Sec_chassis_angle);
    TxMessage.Data[5] = 0;
	TxMessage.Data[6] = 0;// (unsigned char)(gimbal_output[1]);

	mbox = CAN_Transmit(CAN2, &TxMessage);
    i = 0;
    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++;
}

/* CAN2 云台板接收
   云台板：遥控数据（分遥控模式和RC模式，两者不共存），yaw轴电机真实位置，yaw轴电机速度 */
void CAN2_gimbal_receive(CanRxMsg *rx_message)
{
	switch (rx_message->StdId)
	{
        case 0x400:
        {
//			Safecheck_dog.RC_Receive_Flag = 1;
            rc_ctrl.rc.ch[2] = ((rx_message->Data[0] << 8) | rx_message->Data[1]);
            rc_ctrl.rc.ch[3] = ((rx_message->Data[2] << 8) | rx_message->Data[3]);
            rc_ctrl.rc.ch[4] = (rx_message->Data[4] << 8 | rx_message->Data[5]);
			rc_ctrl.rc.s1 = (rx_message->Data[6]) / 10;
            rc_ctrl.rc.s2 = (rx_message->Data[6]) % 10;
            rc_ctrl.mouse.press_l = (rx_message->Data[7]) / 10 - 1;
            rc_ctrl.mouse.press_r = (rx_message->Data[7]) % 10;

            break;
        }
        case 0x401:
        {
//			Safecheck_dog.RC_Receive_Flag = 1;
            rc_ctrl.mouse.x = ((rx_message->Data[0] << 8) | rx_message->Data[1]);
            rc_ctrl.mouse.y = ((rx_message->Data[2] << 8) | rx_message->Data[3]);
            rc_ctrl.key.v = (rx_message->Data[4] << 8 | rx_message->Data[5]);
            rc_ctrl.rc.s1 = (rx_message->Data[6]) / 10;
            rc_ctrl.rc.s2 = (rx_message->Data[6]) % 10;
            rc_ctrl.mouse.press_l = (rx_message->Data[7]) / 10 - 1;
            rc_ctrl.mouse.press_r = (rx_message->Data[7]) % 10;

            break;
        }
        case 0x402:
        {
//			Safecheck_dog.RC_Receive_Flag = 1;
            motor_yaw.actual_Position = (rx_message->Data[0] << 24 | rx_message->Data[1] << 16 | rx_message->Data[2] << 8 | rx_message->Data[3]);
            motor_yaw.speed = (rx_message->Data[4] << 8 | rx_message->Data[5]);
            
            break;
        }
	}
}

/* CAN2 底盘板接收 
   底盘板：云台初始化成功标志,yaw轴中值到达标志，yaw轴电机输出数据，补给状态标志 */
void CAN2_chassis_receive(CanRxMsg *rx_message)
{
	switch (rx_message->StdId)
	{
		case 0x300:
		{
			motor_yaw.actual_Position = (rx_message->Data[3]<<24 | rx_message->Data[2]<<16 | rx_message->Data[1]<<8 | rx_message->Data[0]);
//			INITIALIZE_flag = rx_message->Data[0] / 100;
			fir_gimbal_control.yaw_c.photoelectric_zero = (rx_message->Data[0] % 100) / 10;
			GIMBAL_SUPPLY_FLAG = (rx_message->Data[0]) % 10;
			fir_gimbal_control.yaw_c.output = (rx_message->Data[1] << 8 | rx_message->Data[2]);
			sec_gimbal_control.chassis_different_angle = (rx_message->Data[3] << 8 | rx_message->Data[4]);
			break;
		}
	}

}

