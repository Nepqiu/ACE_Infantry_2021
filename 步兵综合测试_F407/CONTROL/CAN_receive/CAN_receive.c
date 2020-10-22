/**
  ******************************************************************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���
  ******************************************************************************
  */

#include "CAN_Receive.h"
#include "RemoteControl.h"
#include "rmmotor.h"
#include "Task_Gimbal.h"
#include "Task_Chassis.h"

//������ݶ�ȡ
#define get_motor_M3508(ptr, rx_message)                                                           \
        {                                                                                          \
            (ptr)->position = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);      \
            (ptr)->speed = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);         \
        }

		
/*
   ���ּ�д��RC��remote contral   ң����
             MK: mouse key        ����
*/


/*--------------------����-----------------------*/
//�������̵������
motor_measure_t motor_chassis[4];
//���������������
motor_measure_t motor_fire;
//����yaw��3508�������
motor_measure_t motor_yaw;
//��������̨�������
motor_measure_t motor_second_yaw;
//����pitch��������
motor_measure_t motor_pitch;


extern int8_t GIMBAL_SUPPLY_FLAG; //��̨������־λ

/*--------------------����-----------------------*/
//can���մ�����
static void CAN1_gimbal_receive(CanRxMsg *rx_message);
static void CAN1_chassis_receive(CanRxMsg *rx_message);
static void CAN2_gimbal_receive(CanRxMsg *rx_message);
static void CAN2_chassis_receive(CanRxMsg *rx_message);
	
	

//can1�����ж�
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

//can2�����ж�
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
/* CAN1���̰巢�͵����̵�� */
void CAN1_Chassis_SetMsg(int16_t ESC_201, int16_t ESC_202, int16_t ESC_203, int16_t ESC_204)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //����һ��������Ϣ�Ľṹ��

    CAN1_TxMessage.StdId = CAN_CHASSIS_ALL_ID; //����820r���ñ�ʶ��
    CAN1_TxMessage.IDE = CAN_ID_STD;           //ָ����Ҫ�������Ϣ�ı�ʶ��������
    CAN1_TxMessage.RTR = CAN_RTR_DATA;         //ָ����֡�����������Ϣ������   ����֡��Զ��֡
    CAN1_TxMessage.DLC = 8;                    // ������֡��Ϣ
    CAN1_TxMessage.Data[0] = (unsigned char)(ESC_201 >> 8);
    CAN1_TxMessage.Data[1] = (unsigned char)(ESC_201);
    CAN1_TxMessage.Data[2] = (unsigned char)(ESC_202 >> 8);
    CAN1_TxMessage.Data[3] = (unsigned char)(ESC_202);
    CAN1_TxMessage.Data[4] = (unsigned char)(ESC_203 >> 8);
    CAN1_TxMessage.Data[5] = (unsigned char)(ESC_203);
    CAN1_TxMessage.Data[6] = (unsigned char)(ESC_204 >> 8);
    CAN1_TxMessage.Data[7] = (unsigned char)(ESC_204);

    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //������Ϣ
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //�ȴ����ͽ���
        i++;
}

/* CAN1���̰巢�͵���̨Y����͹������ | Y������205��P������206�����������207 */
void CAN1_Chassis_Gimbal_Fire(int16_t ESC_205, int16_t ESC_206, int16_t ESC_207) 
{
	u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //����һ��������Ϣ�Ľṹ��

    CAN1_TxMessage.StdId = 0x1ff;      //����820r���ñ�ʶ��
    CAN1_TxMessage.IDE = CAN_ID_STD;   //ָ����Ҫ�������Ϣ�ı�ʶ��������
    CAN1_TxMessage.RTR = CAN_RTR_DATA; //ָ����֡�����������Ϣ������   ����֡��Զ��֡
    CAN1_TxMessage.DLC = 8;            //ָ�����ݵĳ���
    CAN1_TxMessage.Data[0] = (unsigned char)(ESC_205 >> 8);
    CAN1_TxMessage.Data[1] = (unsigned char)(ESC_205);
		CAN1_TxMessage.Data[2] = (unsigned char)(ESC_206 >> 8);
		CAN1_TxMessage.Data[3] = (unsigned char)(ESC_206);
    CAN1_TxMessage.Data[4] = (unsigned char)(ESC_207 >> 8);
    CAN1_TxMessage.Data[5] = (unsigned char)(ESC_207);
    CAN1_TxMessage.Data[6] = 0;
    CAN1_TxMessage.Data[7] = 0;
    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //������Ϣ
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++; //�ȴ����ͽ���
}

/* CAN1��̨�巢�͵�P���� | P������201������̨yaw2006��202 */
void CAN1_Gimbal_SetMsg(int16_t ESC_201, int16_t ESC_202)
{
    u8 mbox;
    u16 i = 0;
    CanTxMsg CAN1_TxMessage; //����һ��������Ϣ�Ľṹ��

    CAN1_TxMessage.StdId = 0x200;      //����820r���ñ�ʶ��
    CAN1_TxMessage.IDE = CAN_ID_STD;   //ָ����Ҫ�������Ϣ�ı�ʶ��������
    CAN1_TxMessage.RTR = CAN_RTR_DATA; //ָ����֡�����������Ϣ������   ����֡��Զ��֡
    CAN1_TxMessage.DLC = 8;            //ָ�����ݵĳ���
    CAN1_TxMessage.Data[0] = (unsigned char)(ESC_201 >> 8);
    CAN1_TxMessage.Data[1] = (unsigned char)(ESC_201);
    CAN1_TxMessage.Data[2] = (unsigned char)(ESC_202 >> 8);
    CAN1_TxMessage.Data[3] = (unsigned char)(ESC_202);
    CAN1_TxMessage.Data[4] = 0;
    CAN1_TxMessage.Data[5] = 0;
    CAN1_TxMessage.Data[6] = 0;
    CAN1_TxMessage.Data[7] = 0;

    mbox = CAN_Transmit(CAN1, &CAN1_TxMessage); //������Ϣ
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++; //�ȴ����ͽ���
}

/* CAN1���̰���մ�����
   ���̰壺����4������ݷ�����yaw��3508���ݷ������������2006���ݷ�����*/
static void CAN1_chassis_receive(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
		/*���̵��*/
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

		/*yaw��3508����ٶ�*/
		case 0x205:
		{
//			get_motor_M3508(&motor_yaw, rx_message);
			motor_yaw.position = ((rx_message)->Data[0] << 8) | (rx_message)->Data[1];
			Motor_Actual_Position(&motor_yaw, YAW_RATIO, 8192); //����yaw�������ʵ����ֵ
			motor_yaw.speed = ((rx_message)->Data[2] << 8) | (rx_message)->Data[3];
			break;
		}
		
#ifdef fire_work
		/*�������*/
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

/* CAN1��̨����մ�����
   ��̨�壺P�����ٶ�ֵ������P�ᵥȦ������λ�÷���������̨yaw��2006������ݷ���*/
static void CAN1_gimbal_receive(CanRxMsg *rx_message)
{
	switch (rx_message->StdId)
    {
		/*Pitch�������λ�÷���*/
		case 0x01:
		{
			motor_pitch.position = (rx_message->Data[6] << 24 | rx_message->Data[5] << 16 | rx_message->Data[4] << 8 | rx_message->Data[3]);
			motor_pitch.actual_Position = motor_pitch.position - Pitch_Middle_Angle;   //ʵ��ֵ��ȥ�м�ֵ
			break;
		}

		/*P��3510����ٶȷ���*/
		case 0x201:
		{
			motor_pitch.speed = ((rx_message->Data[2] << 8) | rx_message->Data[3]);
			break;
		}

#ifdef double_gimbal
		/*����̨yaw��2006������ݷ���*/
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
/* CAN2 ���̰�ң�������ݷ��� */
void CAN2_Chassis_RC_SetMsg(void)
{
    u8 mbox;
    u16 i = 0;

    CanTxMsg CAN2_TxMessage; //����һ��������Ϣ�Ľṹ��

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
    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF)) //�ȴ����ͽ���
        i++;
}

/* CAN2 ���̰�������ݷ��� */
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

/* ����can2����ң��ֵ2����̨��  Y�������� */
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
    TxMessage.Data[6] = 0;//REFEREE.RobotStatus.shooter_heat0_speed_limit; //�������ƴ�С
    TxMessage.Data[7] = 0;                                             //����

    mbox = CAN_Transmit(CAN2, &TxMessage);
    i = 0;
    while ((CAN_TransmitStatus(CAN2, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
        i++;
}

/* CAN2 ��̨�巢�� */
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

/* CAN2 ��̨�����
   ��̨�壺ң�����ݣ���ң��ģʽ��RCģʽ�����߲����棩��yaw������ʵλ�ã�yaw�����ٶ� */
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

/* CAN2 ���̰���� 
   ���̰壺��̨��ʼ���ɹ���־,yaw����ֵ�����־��yaw����������ݣ�����״̬��־ */
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

