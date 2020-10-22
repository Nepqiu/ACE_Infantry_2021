/**
  ******************************************************************************
  * @file       Task_Chassis.c/h
  * @brief      ��ɵ��̿�������
  ******************************************************************************
  */
#include "Task_Chassis.h"
#include "Task_Detect.h"
#include "RemoteControl.h"
#include "chassis_behaviour.h"
#include "rmmotor.h"


/*--------------------����-----------------------*/
//���̿������� static
chassis_control_t  chassis_control; 

//�����ƶ�����PID
PidTypeDef CHASSIS_MOVE_FOLLOW_PID; 
//������ת����PID
PidTypeDef CHASSIS_ROTATE_FOLLOW_PID; 

#if CHASSIS_TEST_MODE
float lowfilt_ch1_jscope;   //��ͨ�˲�vx��ӡʵ������
float lowfilt_ch0_jscope;   //��ͨ�˲�vy��ӡʵ������
#endif

/*--------------------����-----------------------*/
static void chassis_controlwork(void);
//�������ݳ�ʼ��
static void chassis_init(chassis_control_t *chassis_move_init);
//�������ݸ���	
static void chassis_data_update(void);
//����ң��ģʽѡ��	
static void chassis_remote_mode_choose(void);	
//����pid����
static void chassis_pid_calc(void);	
//���̹�������								
//static void chassis_power_limit(void);
#if CHASSIS_TEST_MODE
/* jscope��ӡ���� */
static void chassis_jscope_print_curve(void);
#endif



void CHASSIS_Task(void *pvParameters)
{
	//����һ��ʱ��
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	//�������ݳ�ʼ��
	chassis_init(&chassis_control);
	
	while(1)
	{
		//���̿���
		chassis_controlwork();
		
		//����ң�������ݸ���̨��
		CAN2_Chassis_RC_SetMsg();
		
		//can���͵���ֵ
		CAN1_Chassis_SetMsg(chassis_control.chassis_motor[0].output, chassis_control.chassis_motor[1].output, chassis_control.chassis_motor[2].output, chassis_control.chassis_motor[3].output);
		
		vTaskDelay(CHASSIS_CONTROL_TIME);
	}
}



//�������ݳ�ʼ��
static void chassis_init(chassis_control_t *chassis_move_init)
{
	//���̿���״̬Ϊֹͣ
	chassis_move_init->chassis_mode = CHASSIS_STOP;
	chassis_move_init->chassis_RC = get_remote_control_point();
	
	//���ң������ֵ�Ƿ���ȷ
	RC_data_is_error();
	
	//��ʼ����ͨ�˲�
	first_order_filter_init(&(chassis_move_init->LowFilt_chassis_vx), CHASSIS_FIRST_ORDER_FILTER_K);
	first_order_filter_init(&(chassis_move_init->LowFilt_chassis_vy), CHASSIS_FIRST_ORDER_FILTER_K);
	
	//���ν����������
	chassis_data_update();
}


static void chassis_controlwork(void)
{
	//���ң������ֵ�Ƿ���ȷ
	RC_data_is_error();
	
#ifdef board_gimbal
	//�����������̨���
	yaw_control.chassis_different_angle = Get_Yaw_Different_Angle(&motor_yaw, YAW_RATIO); 
#endif
	
#ifdef board_chassis
//	if ((rc_ctrl.rc.s2 == 3 && rc_ctrl.rc.s1 == 2) || Vision_flag == 1)
//	{
//		sec_gimbal_control.chassis_different_angle = Sec_chassis_angle; //����̨��Ե��̵Ĳ��
//	}
//	else
//	{
//		yaw_control.chassis_different_angle = Get_Yaw_Different_Angle(&motor_yaw, YAW_RATIO);  //�����������̨���
//	}
	
	//�������ݸ���
	chassis_data_update();
	//ң����ģʽ״̬����
	chassis_remote_mode_choose();
	//���̿���PID����
	chassis_pid_calc();
	
#ifdef power_limit  //��������
	chassis_power_limit(&chassis_control);
#endif

#endif
	
	//�����������̨���
	chassis_control.chassis_gimbal_angel = Get_Yaw_Different_Angle(&motor_yaw, YAW_RATIO);

}


//�������ݸ���
static void chassis_data_update(void)
{
	u8 i;	
	
	//����ٶȸ���
	for(i=0;i<4;i++)
	{
		chassis_control.chassis_motor[i].speed = motor_chassis[i].speed;
	}                                                            
}



//����ң��ģʽѡ��	
static void chassis_remote_mode_choose(void)
{	
	//�ж���ʲôģʽ
	chassis_behaviour_mode_set(&chassis_control);  
}



//���̿���������
void chassis_set_remote(int16_t ch0, int16_t ch1, int16_t ch2)
{
	//һ�׵�ͨ�˲�����
	first_order_filter(&(chassis_control.LowFilt_chassis_vx), -ch0);
	first_order_filter(&(chassis_control.LowFilt_chassis_vy), ch1);
	
	/* ��ӡ���� */
	lowfilt_ch0_jscope = chassis_control.LowFilt_chassis_vx.out;
	lowfilt_ch1_jscope = chassis_control.LowFilt_chassis_vx.out;
	
	/*���������ݵ���ģʽ����*/
    if (work_st == POWEROFF || work_st == INITIALIZE) //�ص���ʼ���׶�
    {
        chassis_control.chassis_motor[0].output = 0;
		chassis_control.chassis_motor[1].output = 0;
		chassis_control.chassis_motor[2].output = 0;
		chassis_control.chassis_motor[3].output = 0;
    }
	else
	{
		//����ֹͣ ���ٶ�Ŀ��ֵ����Ϊ0
		if(chassis_control.chassis_mode == CHASSIS_STOP)
		{
			chassis_control.speed_x_set = 0;
			chassis_control.speed_y_set = 0;
			chassis_control.speed_z_set = 0;
		}
		//���� ��������̨ | Ť�� | С���� ��������ʱ������������������x��y�����z����ȥ���ˣ�
		if(chassis_control.chassis_mode == CHASSIS_NO_FOLLOW || chassis_control.chassis_mode == CHASSIS_TWIST_WAIST || chassis_control.chassis_mode == CHASSIS_ROTATION)
		{
			chassis_control.speed_x_set = chassis_control.LowFilt_chassis_vx.out;
			chassis_control.speed_y_set = chassis_control.LowFilt_chassis_vy.out;
			chassis_control.speed_z_set = ch2;
			
			//�����˶��ֽ�
			chassis_control.chassis_motor[0].speed_set = (-chassis_control.speed_y_set - chassis_control.speed_x_set + chassis_control.speed_z_set) * 5.0f;
			chassis_control.chassis_motor[1].speed_set = ( chassis_control.speed_y_set - chassis_control.speed_x_set + chassis_control.speed_z_set) * 5.0f;
			chassis_control.chassis_motor[2].speed_set = (-chassis_control.speed_y_set + chassis_control.speed_x_set + chassis_control.speed_z_set) * 5.0f;
			chassis_control.chassis_motor[3].speed_set = ( chassis_control.speed_y_set + chassis_control.speed_x_set + chassis_control.speed_z_set) * 5.0f;
		}
		//���� ������̨
		if(chassis_control.chassis_mode == CHASSIS_FOLLOW)
		{
			chassis_control.speed_x_set = chassis_control.LowFilt_chassis_vx.out;
			chassis_control.speed_y_set = chassis_control.LowFilt_chassis_vy.out;
			chassis_control.speed_z_set = ch2;
			
			//�����˶��ֽ�
			chassis_control.chassis_motor[0].speed_set = (-chassis_control.speed_y_set - chassis_control.speed_x_set + chassis_control.speed_z_set) * 2.0f;
			chassis_control.chassis_motor[1].speed_set = ( chassis_control.speed_y_set - chassis_control.speed_x_set + chassis_control.speed_z_set) * 2.0f;
			chassis_control.chassis_motor[2].speed_set = (-chassis_control.speed_y_set + chassis_control.speed_x_set + chassis_control.speed_z_set) * 2.0f;
			chassis_control.chassis_motor[3].speed_set = ( chassis_control.speed_y_set + chassis_control.speed_x_set + chassis_control.speed_z_set) * 2.0f;

		}
	}
	
#if CHASSIS_TEST_MODE
	chassis_jscope_print_curve();
#endif
}


//���̿���PID����
static void chassis_pid_calc(void)
{
//	chassis_control.chassis_motor[0].output = PID_Calc(&(chassis_control.chassis_speed_pid[0]), chassis_control.chassis_motor[0].speed, chassis_control.chassis_motor[0].speed_set);
//	chassis_control.chassis_motor[1].output = PID_Calc(&(chassis_control.chassis_speed_pid[1]), chassis_control.chassis_motor[1].speed, chassis_control.chassis_motor[1].speed_set);
//	chassis_control.chassis_motor[2].output = PID_Calc(&(chassis_control.chassis_speed_pid[2]), chassis_control.chassis_motor[2].speed, chassis_control.chassis_motor[2].speed_set);
//	chassis_control.chassis_motor[3].output = PID_Calc(&(chassis_control.chassis_speed_pid[3]), chassis_control.chassis_motor[3].speed, chassis_control.chassis_motor[3].speed_set);
	
	/*���PID�ٶȱջ�����*/
	chassis_control.chassis_motor[0].output = Rmmotor_Speed_control(&(chassis_control.chassis_speed_pid[0]), chassis_control.chassis_motor[0].speed_set, chassis_control.chassis_motor[0].speed, M3508_MAX_OUTPUT_CURRENT);
	chassis_control.chassis_motor[1].output = Rmmotor_Speed_control(&(chassis_control.chassis_speed_pid[1]), chassis_control.chassis_motor[1].speed_set, chassis_control.chassis_motor[1].speed, M3508_MAX_OUTPUT_CURRENT);
	chassis_control.chassis_motor[2].output = Rmmotor_Speed_control(&(chassis_control.chassis_speed_pid[2]), chassis_control.chassis_motor[2].speed_set, chassis_control.chassis_motor[2].speed, M3508_MAX_OUTPUT_CURRENT);
	chassis_control.chassis_motor[3].output = Rmmotor_Speed_control(&(chassis_control.chassis_speed_pid[3]), chassis_control.chassis_motor[3].speed_set, chassis_control.chassis_motor[3].speed, M3508_MAX_OUTPUT_CURRENT);                                          

}



#ifdef power_limit
//���̹�������	
static void chassis_power_limit(chassis_control_t *chassis_power_limit)
{

}
#endif



#if CHASSIS_TEST_MODE
/* jscope��ӡ���� */
static void chassis_jscope_print_curve(void)
{
	
	lowfilt_ch0_jscope = chassis_control.LowFilt_chassis_vx.out;
	lowfilt_ch1_jscope = chassis_control.LowFilt_chassis_vx.out;
}
#endif











