#ifndef __REMOTETASK_H
#define __REMOTETASK_H
#include "main.h"

/*OS�������������Լ�����ʱ��*/
#define REMOTE_TASK_INIT_TIME   5    //ң��������������ʱ��
#define REMOTE_CONTROL_TIME_MS  7      //ң������ѭ��ʱ��


//������ң������
typedef __packed struct
{
    float RC_ch0;
    float RC_ch1;
    float RC_ch2;
    float RC_ch3;
    float RC_sw;
} RC_Deal_data;




/*��������*/
void REMOTE_Task(void *pvParameters);    //ң������
void Select_Ctl_Mode(void);              //ģʽѡ��
void RC_Data_Process(void);              //ң�������ݴ���
//void MK_Data_Process(void);              //�������ݴ���
void Remote_reload(void);                //ҡ��������


/*ң�ش���ֵ*/
extern RC_Deal_data        Remote_data;            //ң�ش�����ֵ



#endif
