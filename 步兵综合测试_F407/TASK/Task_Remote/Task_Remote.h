#ifndef __REMOTETASK_H
#define __REMOTETASK_H
#include "main.h"

/*OS控制任务周期以及启动时间*/
#define REMOTE_TASK_INIT_TIME   5    //遥控任务启动缓冲时间
#define REMOTE_CONTROL_TIME_MS  7      //遥控任务循环时间


//处理后的遥控数据
typedef __packed struct
{
    float RC_ch0;
    float RC_ch1;
    float RC_ch2;
    float RC_ch3;
    float RC_sw;
} RC_Deal_data;




/*函数声明*/
void REMOTE_Task(void *pvParameters);    //遥控任务
void Select_Ctl_Mode(void);              //模式选择
void RC_Data_Process(void);              //遥控器数据处理
//void MK_Data_Process(void);              //键盘数据处理
void Remote_reload(void);                //摇杆量清零


/*遥控处理值*/
extern RC_Deal_data        Remote_data;            //遥控处理后的值



#endif
