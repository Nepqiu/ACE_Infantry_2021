#include "gimbal_behaviour.h"
#include "Task_Gimbal.h"
#include "Task_Remote.h"
#include "maths.h"
#include "rmmotor.h"
#include "filter.h"
#include "CAN_Receive.h"
#include "IMU.h"
#include "photoelectric.h"
#include "Task_Chassis.h"
#include "Task_Detect.h"
#include "RemoteControl.h"
#include "chassis_behaviour.h"

//��̨��ʼ��
static void Gimbal_Location_Init(gimbal_first_control_t *fir_gimbal_location_init, gimbal_second_control_t *sec_gimbal_location_init);             
//��̨ģʽѡ��
static void gimbal_mode_choose(gimbal_first_control_t *fir_gimbal_mode_choose, gimbal_second_control_t *sec_gimbal_mode_choose);
//Yaw���ʼ��
static void Yaw_Init(gimbal_first_control_t *yaw_location_init);
//��Yaw���ʼ��
static void Sec_Yaw_Init(gimbal_second_control_t *sec_yaw_location_init);           
//Pitch���ʼ��
static void Pitch_Init(gimbal_first_control_t *pitch_location_init);              
//��̨����
static void Gimbal_Stop(gimbal_first_control_t *fir_gimbal_stop, gimbal_second_control_t *sec_gimbal_stop);             


static float Gimbal_ch2 = 0.0f, Gimbal_ch3 = 0.0f; //��̨����ܿ���


void Gimbal_behaviour_mode_set(gimbal_first_control_t *fir_gimbal_behaviour, gimbal_second_control_t *sec_gimbal_behaviour)
{
	/* ģʽѡ�� */
	gimbal_mode_choose(fir_gimbal_behaviour, sec_gimbal_behaviour);
	
	/* ״̬�л� */
    if (Gimbal_all_init_detection() == 0)
    {
        Gimbal_Location_Init(fir_gimbal_behaviour, sec_gimbal_behaviour); //��̨��ʼ��
    }
	/* ��Ϊ�ֶ�ģʽworking */
    if ((fir_gimbal_behaviour->fir_gimbal_mode == GIMBAL_WORKING) && (sec_gimbal_behaviour->sec_gimbal_mode == GIMBAL_WORKING))
    {
		Gimbal_ch2 += (rc_ctrl.rc.ch[2]) * RC_YAW_SPEED;  //Y��λ�û����ۼ�  0.021  RC_YAW_SPEED
        Gimbal_ch2 = loop_fp32_constrain(Gimbal_ch2, -180.0f, 180.0f); //ѭ���޷�
        Gimbal_ch3 += (rc_ctrl.rc.ch[3]) * 0.04f;  //P��λ�û����ۼ�  0.08  RC_PITCH_SPEED
		
        Gimbal_Work(fir_gimbal_behaviour, Gimbal_ch2, Gimbal_ch3);
    }
    else
    {
        Gimbal_Stop(fir_gimbal_behaviour, sec_gimbal_behaviour); //ֹͣ
    }
}

static void gimbal_mode_choose(gimbal_first_control_t *fir_gimbal_mode_choose, gimbal_second_control_t *sec_gimbal_mode_choose)
{
	//�ҿ��ش��ϣ�ң�ؿ���
    if (rc_ctrl.rc.s2 == RC_SW_UP) 
    {
//		/* �ж�״̬�Ƿ���� */
//		if (s2_location == RC_SW_MID) //��һ��״̬Ϊs2���У����»�����״̬
//		{
//			chassis_control.chassis_mode = CHASSIS_FOLLOW; //����Ĭ�Ϲ���ģʽΪ����
//			Remote_reload();  //ҡ��������
//		}

		/* �ж�֮ǰ��״̬Ϊ ʲô */
        if (work_st == POWEROFF) //֮ǰ��״̬Ϊ����
        {
            work_st = INITIALIZE; //״̬����Ϊ��ʼ��
            Remote_reload();      //ҡ��������
        }
        else if (work_st == AUTOATTACK || work_st == AUTOBUFF) //֮ǰ״̬Ϊ�Զ��������ģʽ
        {
            work_st = WORKING; //״̬����Ϊ����
        }
        else /* ��ʼ��״̬ �ֶ�״̬ ����״̬ ˫��̨״̬ */
        {
            if (work_st != INITIALIZE) //״̬��Ϊ��ʼ��
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
		work_st = POWEROFF;    //�������ģʽ
//		INITIALIZE_flag = 0;   //��ʼ����־λ����
        Remote_reload();       //ҡ��������
    }
	//�ҿ��ش��£�ֹͣ����
    if (rc_ctrl.rc.s2 == RC_SW_DOWN) 
    {
        work_st = POWEROFF;    //�������ģʽ
//		INITIALIZE_flag = 0;   //��ʼ����־λ����
        Remote_reload();       //ҡ��������
    }
//	if (rc_ctrl.rc.s2 == 0)
//	{
//		work_st = POWEROFF;    //�������ģʽ
//		INITIALIZE_flag = 0;   //��ʼ����־λ����
//		Remote_reload();       //ҡ��������
//	}
}


/*==============================��̨��ʼ��==============================*/
//��̨��ʼ����
static void Gimbal_Location_Init(gimbal_first_control_t *fir_gimbal_location_init, gimbal_second_control_t *sec_gimbal_location_init)
{
	fir_gimbal_location_init->yaw_c.init_flag = 1;
	fir_gimbal_location_init->pitch_c.init_flag = 1;
	sec_gimbal_location_init->init_flag = 1;
	
//	set_gimbal_initflag(yaw_control, pitch_control, sec_gimbal_control, 1);

//	Yaw_Init();     //Y���ʼ��
//	Pitch_Init();   //P���ʼ��
//	Sec_Yaw_Init(); //��Y���ʼ��
	
	Sliding_Mean_Filter_Init(&fir_gimbal_location_init->pitch_c.Slidmean_Pitch_Data); //��ʼ��P�Ử���˲���

    if (fir_gimbal_location_init->yaw_c.init_flag == 1 && fir_gimbal_location_init->pitch_c.init_flag == 1 && sec_gimbal_location_init->init_flag == 1)
    {
        fir_gimbal_location_init->yaw_c.init_flag = 0;
		fir_gimbal_location_init->pitch_c.init_flag = 0;
		sec_gimbal_control.init_flag = 0;
//        INITIALIZE_flag = 1;
        work_st = WORKING; //��ʼ����
    }
}

//Yaw���ʼ��
static void Yaw_Init(gimbal_first_control_t *yaw_location_init)
{
    if (yaw_location_init->yaw_c.init_flag == 0) //Y���ʼ��δ�ɹ�
    {
		//�ٶȻ���900Ŀ��ֵ
        yaw_location_init->yaw_c.output = Rmmotor_Speed_control(&yaw_location_init->yaw_c.yaw_s_pid, 900, motor_yaw.speed, 1500);
        
		//���̽ӽ���ֵ
		if (yaw_location_init->yaw_c.photoelectric_zero == 1)
        {
            yaw_location_init->yaw_c.init_flag = 1;
        }
    }
    else
    {
        yaw_location_init->yaw_c.output = 0;
    }
}

//Pitch���ʼ��
static int16_t pitch_init_error = 0; //P���ʼ����ʧ�ܴ���
static int16_t pitch_init_count = 0; //P���ʼ����̨������������
static void Pitch_Init(gimbal_first_control_t *pitch_location_init)
{
	if (pitch_location_init->pitch_c.init_flag == 0) //P��δ��ʼ�����
    {
		//λ�û�Ŀ��ֵΪ0
		pitch_location_init->pitch_c.output = Motor_Position_Speed_Control(&pitch_location_init->pitch_c.pitch_s_pid, &pitch_location_init->pitch_c.pitch_p_pid, 0, motor_pitch.speed, (motor_pitch.actual_Position * 360 / 1024), PITCH_OUTPUT_LIMIT);                                       

		if (abs(Pitch_Middle_Angle - motor_pitch.position) < 20) //���̽ӽ���ֵ
		{
			if (pitch_init_count >= 50) //��ʼ������
			{
				pitch_location_init->pitch_c.init_flag = 1;
				pitch_init_count = 0; //��λ
				pitch_init_error = 0;
			}
			else
			{
				pitch_init_count++; //�ɹ������ۼ�
			}
		}
		else
		{
			pitch_init_error++; //��ʼ��ʧ�ܼ�¼
		}

        if (pitch_init_error == 1500) //��γ�ʼ��ʧ�ܣ�ǿ�Ƴ�ʼ���ɹ�
        {
            pitch_location_init->pitch_c.init_flag = 1;
            pitch_init_count = 0; //��λ
            pitch_init_error = 0;
        }
    }
    else //P���ʼ�����
    {
        pitch_location_init->pitch_c.output = Motor_Position_Speed_Control
		                                      (&pitch_location_init->pitch_c.pitch_s_pid, 
		                                      &pitch_location_init->pitch_c.pitch_p_pid, 
		                                      0, 
		                                      motor_pitch.speed, 
		                                      (motor_pitch.actual_Position * 360 / 1024), 
		                                      PITCH_OUTPUT_LIMIT);
    }
}

//����̨yaw���ʼ��
static void Sec_Yaw_Init(gimbal_second_control_t *sec_yaw_location_init)
{
	
}



/*==================================��̨����==================================*/
static void Gimbal_Stop(gimbal_first_control_t *fir_gimbal_stop, gimbal_second_control_t *sec_gimbal_stop)
{
    fir_gimbal_stop->pitch_c.output = 0;
    fir_gimbal_stop->yaw_c.output = 0;
    sec_gimbal_control.output = 0;
}




