/*************��ȫ���*******************
���ݣ�1.ң������ȫ��⣨���󣺶����������쳣
                      ����ǿ�б�Ϊ����״̬��
      2.can2������⣨���󣺶����������쳣
                     ������̨�壺�ָ�����״̬
						   ���̰壺�ָ�����״̬��
��ע��can���Ź���û��
*****************************************/
#include "Task_Safecheck.h"
#include "Task_Detect.h"
#include "Task_Chassis.h"
#include "Task_Gimbal.h"
#include "Task_Safecheck.h"
#include "RemoteControl.h"


//================��������
int safecheck_heart = 0; //��ȫ�����������

//================�ṹ�嶨��
DogTypeDef Safecheck_dog = {0, 0, 0}; //��ؽṹ������
DogTypeDef Dog = {0, 1, 1};

//================�ڲ���������
static void RC_Check(void);
static void Dog_Feed(void);

/*=======��ȫ�������======*/
void SAFECHECK_Task(void *pvParameters)
{

    while (1)
    {
#ifdef watch_dog
        //ң�ض������
        RC_Check();
        //ι��
        Dog_Feed();
#endif

        //�������
        vTaskDelay(CHECK_CONTROL_TIME_MS / portTICK_RATE_MS);  // portTICK_RATE_MS ���ڽ�������Ϊ��λ��ʱ��ֵת��Ϊ�Ժ���Ϊ��λ��ʱ��ֵ��
    }
}

/*=================���ң������========================*/
static void RC_Check(void)
{
    /*������¼*/
    if (Safecheck_dog.RC_Receive_Flag == 0 || rc_ctrl.rc.s1 < 0 || rc_ctrl.rc.s1 > 3 || rc_ctrl.rc.s2 < 0 || rc_ctrl.rc.s2 > 3) //��ң�����ݷ��ػ�ң�����ݳ��ִ���
    {
        Safecheck_dog.RC_Error++;
        if (Safecheck_dog.RC_Error >= 100)
            Safecheck_dog.RC_Error = 100;
    }
    else
    {
        Safecheck_dog.RC_Error = 0;
    }

    /*������־λ����*/
    Safecheck_dog.RC_Receive_Flag = 0; //����ң�ؽ��ձ�־λ

    /*��������*/
    if (Safecheck_dog.RC_Error >= 20) //����400ms
    {
        rc_ctrl.rc.s2 = 2;               //�������ģʽ
//        INITIALIZE_flag = 0;            //��ʼ����־λ����
        Safecheck_dog.RC_Connected = 0; //ң��ȷ������
    }
    else
    {
        Safecheck_dog.RC_Connected = 1; //ң��������
    }
}

/*=======================ι������=======================*/
static void Dog_Feed(void)
{
    if (Safecheck_dog.RC_Connected == 1)
    {
        IWDG_ReloadCounter(); //��װ��
    }
}
