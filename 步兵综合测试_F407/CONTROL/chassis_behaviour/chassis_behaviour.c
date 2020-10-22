/**
******************************************************************************
* @file       chassis_behaviour.c/h
* @brief      ����״̬����
******************************************************************************
*/
#include "chassis_behaviour.h"
#include "Task_Chassis.h"
#include "Task_Gimbal.h"
#include "Task_Detect.h"
#include "RemoteControl.h"
#include "chassis_behaviour.h"
#include "gimbal_behaviour.h"
#include "rmmotor.h"
#include "Task_Remote.h"


static void chassis_mode_choose(chassis_control_t *chassis_mode_set);  //����ģʽѡ��
static void Chassis_Follow(chassis_control_t *Chassis_Follow_control); //���̸���
static void Chassis_Twist(chassis_control_t *Chassis_Twist_control);   //����Ť��
static void Chassis_Rotation(chassis_control_t *Chassis_Rotation_control);        //����С����
static void Chassis_Independent(chassis_control_t *Chassis_Independent_control);  //���̲�����


static float Chassis_ch0 = 0.0f, Chassis_ch1 = 0.0f, Chassis_ch2 = 0.0f; //���̵���ܿ���



//����ģʽ
void chassis_behaviour_mode_set(chassis_control_t *chassis_behaviour)
{
//	chassis_behaviour->chassis_mode = CHASSIS_FOLLOW;
	chassis_behaviour->chassis_mode = CHASSIS_NO_FOLLOW;
//	work_st = WORKING;
	
	/* ����ģʽѡ�� */
	chassis_mode_choose(chassis_behaviour);
	
	//ֹͣ
	if(chassis_behaviour->chassis_mode == CHASSIS_STOP)
	{
		Remote_reload();  //ң������ֵ����
	}
	//����
	if(chassis_behaviour->chassis_mode == CHASSIS_FOLLOW)
	{
		Chassis_Follow(chassis_behaviour); //����
	}
	//������
	if(chassis_behaviour->chassis_mode == CHASSIS_NO_FOLLOW)
	{
		Chassis_Independent(chassis_behaviour); //���̲�����
	}
	//Ť��
	if(chassis_behaviour->chassis_mode == CHASSIS_TWIST_WAIST)
	{
		Chassis_Twist(chassis_behaviour); //Ť��
	}
	//С����
	if(chassis_behaviour->chassis_mode == CHASSIS_ROTATION)
	{
		Chassis_Rotation(chassis_behaviour); //С����
	}
}

/*======����ģʽѡ��========*/
static void chassis_mode_choose(chassis_control_t *chassis_mode_set)
{
	//�ҿ��ش��ϣ�ң�ؿ���
    if (rc_ctrl.rc.s2 == RC_SW_UP) 
    {
		/* �ж�֮ǰ��״̬��ʲô */
        if (chassis_mode_set->last_chassis_mode == CHASSIS_POWEROFF) //֮ǰ��״̬Ϊ����
        {
            chassis_mode_set->chassis_mode = CHASSIS_INITIALIZE; //״̬����Ϊ��ʼ��
            Remote_reload();      //ҡ��������
        }
        else /* ��ʼ��״̬ �ֶ�״̬ ����״̬ ˫��̨״̬ */
        {
			/* �����̨Yaw�ᡢPitch�ᡢ����̨��ʼ�����û�� */
            if (/*(chassis_mode_set->chassis_mode != CHASSIS_INITIALIZE) && */Gimbal_all_init_detection()) //״̬��Ϊ��ʼ��
            {
				chassis_mode_set->chassis_mode = CHASSIS_WORKING; //����Ϊ����ģʽ�����Թ�����
            }
			if(chassis_mode_set->chassis_mode == CHASSIS_WORKING)
			{
				RC_Data_Process(); //ң�������ݴ���
				
                /*�����л�ģʽ*/
                if (rc_ctrl.rc.s1 == RC_SW_UP) //s1�����ϣ�����
                {
                    chassis_control.chassis_mode = CHASSIS_ROTATION; //����С����ģʽ   CHASSIS_ROTATION
                }
                if (rc_ctrl.rc.s1 == RC_SW_MID) //s1�����ϣ����м�
                {
                    chassis_control.chassis_mode = CHASSIS_FOLLOW; //���̸���ģʽ  ƽ��ң��״̬  CHASSIS_FOLLOW
                }
                if (rc_ctrl.rc.s1 == RC_SW_DOWN) //s1�����ϣ�������
                {
                    chassis_control.chassis_mode = CHASSIS_TWIST_WAIST; //����Ť��ģʽ   CHASSIS_TWIST_WAIST
                }
			}
        }
    }
	//�ҿ��ش��У����̿���
    if (rc_ctrl.rc.s2 == RC_SW_MID) 
    {
		chassis_mode_set->last_chassis_mode = CHASSIS_STOP; //��ʱû�м��̿���
        Remote_reload();       //ҡ��������
    }
	//�ҿ��ش��£�ֹͣ����
    if (rc_ctrl.rc.s2 == RC_SW_DOWN) 
    {
		chassis_mode_set->last_chassis_mode = CHASSIS_POWEROFF;  //�������״̬
        Remote_reload();       //ҡ��������
    }
//	if (rc_ctrl.rc.s2 == 0)
//	{
//		work_st = POWEROFF;    //�������ģʽ
//		Remote_reload();       //ҡ��������
//	}
}

/*====���̸���״̬����====*/
static void Chassis_Follow(chassis_control_t *Chassis_Follow_control)
{
//	Chassis_ch2 += ((Chassis_Follow_control->chassis_RC->rc.ch[2]) * RC_YAW_SPEED);  //Y��λ�û����ۼ�  0.021  RC_YAW_SPEED
//	Chassis_ch2 = loop_fp32_constrain(Chassis_ch2, -180.0f, 180.0f);
	
	Chassis_ch0 = Chassis_Follow_control->chassis_RC->rc.ch[0];
	Chassis_ch1 = Chassis_Follow_control->chassis_RC->rc.ch[1];
	
	//��ȡ��̨����̵Ĳ��
    Chassis_ch2 = chassis_control.chassis_gimbal_angel;
	
	//��������̵�pidĿ��ֵ�趨Ϊ���
    CHASSIS_ROTATE_FOLLOW_PID.SetValue = Chassis_ch2;
	
	//������pid��������ֵ���� Chassis_ch2
    Chassis_ch2 = Location_Pid_Int32(&CHASSIS_ROTATE_FOLLOW_PID, 0);
	
	//���̸��棬��̨��ǿ���ch2ת��
    chassis_set_remote(Chassis_ch0, Chassis_ch1, Chassis_ch2); 
}

/*======���̲�����״̬����========*/
static void Chassis_Independent(chassis_control_t *Chassis_Independent_control)
{
//	//*�Ƶ׷����ƶ��㷨����̨Ϊ�������ᣬĿ��ֵ�ֽ⵽���������ᣨ��С����һ����
//	if (Remote_data.RC_ch1 || Remote_data.RC_ch0)
//	{
//		Chassis_ch1 = cos(Yaw_different_angle / 180 * PI) * Remote_data.RC_ch1; //��Ҫ�û��ȵ�λ����
//		Chassis_ch0 = -sin(Yaw_different_angle / 180 * PI) * Remote_data.RC_ch1;
//		if (Remote_data.RC_ch0)
//		{
//			Chassis_ch1 += sin(Yaw_different_angle / 180 * PI) * Remote_data.RC_ch0;
//			Chassis_ch0 += cos(Yaw_different_angle / 180 * PI) * Remote_data.RC_ch0;
//		}
//	}
//	else
//	{
//		Chassis_ch0 = 0;
//		Chassis_ch1 = 0;
//	}
	
	Chassis_ch0 = Chassis_Independent_control->chassis_RC->rc.ch[0];
	Chassis_ch1 = Chassis_Independent_control->chassis_RC->rc.ch[1];

    //*����ֵ����
    chassis_set_remote(Chassis_ch0, Chassis_ch1, 0);
}

/*===����Ť��״̬����=====*/
static void Chassis_Twist(chassis_control_t *Chassis_Twist_control)
{
	
}

/*=====����С����״̬����======*/
static void Chassis_Rotation(chassis_control_t *Chassis_Rotation_control)
{
    /* С������ת�ƶ��㷨����̨Ϊ�������ᣬĿ��ֵ�ֽ⵽���������� */
    if (Remote_data.RC_ch1 || Remote_data.RC_ch0) //��ʱ���ƶ�
    {
        Chassis_ch2 = CHASSIS_ROTATION_MOVE_SPEED; //С�����ƶ�ʱ����

        Chassis_ch1 = cos(chassis_control.chassis_gimbal_angel / 180 * PI) * Remote_data.RC_ch1; //��Ҫ�û��ȵ�λ����
        Chassis_ch0 = -sin(chassis_control.chassis_gimbal_angel / 180 * PI) * Remote_data.RC_ch1;
        if (Remote_data.RC_ch0)
        {
            Chassis_ch1 += sin(chassis_control.chassis_gimbal_angel / 180 * PI) * Remote_data.RC_ch0;
            Chassis_ch0 += cos(chassis_control.chassis_gimbal_angel / 180 * PI) * Remote_data.RC_ch0;
        }
    }
	/* ԭ��ת���� */
    else 
    {
        //*�����̶�ת��
        Chassis_ch2 = CHASSIS_ROTATION_SPEED; //�������Ҹ���
        Chassis_ch0 = 0;
        Chassis_ch1 = 0;
    }

    //*����ֵ����
    chassis_set_remote(Chassis_ch0*2, Chassis_ch1*2, Chassis_ch2);
}




