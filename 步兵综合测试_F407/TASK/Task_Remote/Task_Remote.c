/**
  ******************************************************************************
  * @file       Task_Remote.c/h
  * @brief      ���ң�������ݴ���
  ******************************************************************************
  */
#include "Task_Remote.h"
#include "RemoteControl.h"
#include "Task_Start.h"
#include "Task_Detect.h"
#include "Task_Chassis.h"
#include "Task_Gimbal.h"
#include "Task_Safecheck.h"


/*****************************������λ˵����**********************************************************/
/* һ��ң��ģʽ��
         1.���̸���  ����������
         2.Ť��ģʽ  ����������
         3.����С���ݣ���������
         4.���ģʽ  ���������У���������̨�������ƶ��������棩
         5.����ģʽ  ����������
         6.����ģʽ  ����������
         7.����      ���ڳ�ʼ��������ҿ��ش��ϻ�������£����Ͻǲ��������Ϸ���������ֹͣ�䵯
         8.�ػ�      ������				 
	 ��������ģʽ��
         1.�����˶���WASD
         2.��̨�˶������
         3.���䣺    ���������㵥������ס����
         4.���٣����ݷŵ磩��    ��סshift����Ϻ����
         5.Ť��ģʽ�� F  ����һ�½��룬�ٰ�һ�·��أ�
         6.����ģʽ�� ����Ҽ�  ����ס��
         7.����ģʽ��  G  ����һ����̨��һ��ת�����굯�ٰ�һ�»�����
         8.������ģʽ��C ����һ�ν��룩�����ģʽû���ˣ�֮ǰȫ��������ٴ�
         9.������ģʽ��V ����һ�ν��룩
         10.�˵�ģʽ�� Z  ����ס��
         11.��̨ģʽ�� Ctrl ����ס��ֻ�ܿ�����̨�����̲�����
         12.���ģʽ�� X ��һ�ν��룩
         13.С����ģʽ��R����һ�ν��룩
		 
ÿ���л�״̬�����Ƚ������ģʽ��Ҳ�������³�ʼ��
                                                                                                    */
/****************************************************************************************************/
/*
   ���ּ�д��RC��remote contral   ң����
             MK: mouse key        ����
*/

/*ң���������Ĵ���ֵ*/
RC_Deal_data Remote_data;
/*��ջʣ��*/
extern TaskHandle_t RemoteTask_Handler; // ջ��С

static u8 s2_location = 0;     //�ñ���Ϊ��¼�ô�ң��ģʽ״̬��s2��
//extern int8_t INITIALIZE_flag; //���͸����̵ĳ�ʼ���ɹ���־
extern int gimbal_work_mode;   //��̨״̬�л������������־λ


void REMOTE_Task(void *pvParameters)
{
    //����ʱ��
    vTaskDelay(REMOTE_TASK_INIT_TIME);

    while (1)
    {
		/* ״̬����Ϊ���� �����Ͳ��ý����ʼ����ֱ�ӿ��Բ�����s1��s2����Ҳ������ ����ֱ��ע�͵��ͺ��� */
		work_st = WORKING; 
		
        Select_Ctl_Mode(); //ң�����ݴ���
		
        vTaskDelay(REMOTE_CONTROL_TIME_MS); //��������
    }
}

/* ң��ѡ��ģʽ���� */
void Select_Ctl_Mode()
{
	//�ҿ��ش��ϣ�ң�ؿ���
    if (rc_ctrl.rc.s2 == RC_SW_UP) 
    {
		/* �ж�״̬�Ƿ���� */
        if (s2_location == RC_SW_MID) //��һ��״̬Ϊs2���У����»�����״̬
        {
            chassis_control.chassis_mode = CHASSIS_FOLLOW; //����Ĭ�Ϲ���ģʽΪ����
            Remote_reload();  //ҡ��������
        }

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
        s2_location = RC_SW_UP; //s2Ϊ��
    }
	//�ҿ��ش��У����̿���
    if (rc_ctrl.rc.s2 == RC_SW_MID) 
    {
		work_st = POWEROFF;    //�������ģʽ
//        INITIALIZE_flag = 0;   //��ʼ����־λ����
        s2_location = RC_SW_MID; //s2����
        Remote_reload();       //ҡ��������
    }
	//�ҿ��ش��£�ֹͣ����
    if (rc_ctrl.rc.s2 == RC_SW_DOWN) 
    {
        work_st = POWEROFF;    //�������ģʽ
//        INITIALIZE_flag = 0;   //��ʼ����־λ����
        s2_location = RC_SW_DOWN; //s2����
        Remote_reload();       //ҡ��������
    }
    if (rc_ctrl.rc.s2 == 0)
    {
        work_st = POWEROFF;    //�������ģʽ
//        INITIALIZE_flag = 0;   //��ʼ����־λ����
        Remote_reload();       //ҡ��������
    }
}

/* ң�������ݴ��� */
void RC_Data_Process(void)
{
//	rc_deadline_limit(rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[0], 10);  //��������
//	rc_deadline_limit(rc_ctrl.rc.ch[1], rc_ctrl.rc.ch[1], 10);  //��������
//	rc_deadline_limit(rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[2], 10);  //��������
//	rc_deadline_limit(rc_ctrl.rc.ch[3], rc_ctrl.rc.ch[3], 10);  //��������
	
    /*��ң����ֵ����ң����*/
    if (work_st == WORKING || work_st == AUTOATTACK || work_st == AUTOBUFF) //�ֶ�����
    {
        Remote_data.RC_ch0 = (rc_ctrl.rc.ch[0]);  //ch0
        Remote_data.RC_ch1 = (rc_ctrl.rc.ch[1]);  //ch1
		
        Remote_data.RC_ch2 += (rc_ctrl.rc.ch[2]) * RC_YAW_SPEED;  //Y��λ�û����ۼ�  0.021  RC_YAW_SPEED
        Remote_data.RC_ch2 = loop_fp32_constrain(Remote_data.RC_ch2, -180.0f, 180.0f); //ѭ���޷�
        Remote_data.RC_ch3 += (rc_ctrl.rc.ch[3]) * 0.04f;  //P��λ�û����ۼ�  0.08  RC_PITCH_SPEED
		
        Remote_data.RC_sw = -(rc_ctrl.rc.ch[4]);  //ң��������ֵ����ֵΪ1024������������ࡣ�������ֵΪ0
    }
    else //����ģʽ����ʼ��ģʽ������ģʽ  �������ֶ�����
    {
        Remote_reload(); //ҡ��������
    }
}



/* ҡ�������� */
void Remote_reload(void)
{
    Remote_data.RC_ch0 = 0;
    Remote_data.RC_ch1 = 0;
    Remote_data.RC_ch2 = 0;
    Remote_data.RC_ch3 = 0;
    Remote_data.RC_sw = 0;
	
//	rc_ctrl.rc.ch[0] = 0;
//	rc_ctrl.rc.ch[1] = 0;
//	rc_ctrl.rc.ch[2] = 0;
//	rc_ctrl.rc.ch[3] = 0;
//	rc_ctrl.rc.ch[4] = 0;
//	rc_ctrl.rc.s1 = RC_SW_MID;
//	rc_ctrl.rc.s2 = RC_SW_DOWN;
//	rc_ctrl.mouse.x = 0;
//	rc_ctrl.mouse.y = 0;
//	rc_ctrl.mouse.z = 0;
//	rc_ctrl.mouse.press_l = 0;
//	rc_ctrl.mouse.press_r = 0;
//	rc_ctrl.key.v = 0;
}
