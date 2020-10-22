#ifndef __MODEDEF_H
#define __MODEDEF_H

/* 步兵控制的宏定义 */


/*模块工作属性*/
//#define watch_dog                //启动看门狗
#define gimbal_work              //云台工作
#define chassis_work             //底盘工作
#define fire_work               //射弹模式开启 (开摩擦轮)
//#define power_limit             //启动功率限制
//#define double_gimbal           //使用双云台
//#define super_capacitor         //使用超级电容
//#define chassis_exclusive_use   //底盘裸机使用


//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE    1
#define CHASSIS_TEST_MODE   1

#define IMU_BMI160  0             //陀螺仪工作 1为使用


/* 主控板类型 */
#define board_chassis       //board_gimbal//board_chassis     //板子是云台板还是底盘板


/*****各机器人的云台中值(如果用到编码器校准，目前2020赛季步兵P轴用到编码器初始化)******/
#define Pitch_Middle_Angle  190  //步兵1:6052   步兵2:1530
#define Pitch_UP_Angle      190  //步兵1:6052   步兵2:1530	


/*****陀螺仪方向参数配置*****/
#define PITCH_GYRO_ANGULAR_SPEED  0//-(MPU6050_Real_Data.Gyro_Y) //P轴角速度
#define YAW_GYRO_ANGULAR_SPEED    0//-(MPU6050_Real_Data.Gyro_Z) //Y轴角速度       
#define YAW_POSTION_OUTPUT_FLAG   (1) 
#define YAW_ANGLE_FLAG            (1)  //陀螺仪位置对Y轴角度的影响
#define YAW_SPEED_OUTPUT_FLAG     (-1)  //纯速度环Y电机速度方向
#define PITCH_POSTION_OUTPUT_FLAG (-1)
#define PITCH_SPEED_OUTPUT_FLAG   (-1)


///*****测试普通自瞄相机选择PID参数*********/
//#define test_short_focus  1      //短焦相机
//#define test_long_focus   0      //长焦相机
//#define test_industry     0      //工业相机

/*************云台pitch和yaw角度限制************/
#define PITCH_ANGLE_LIMIT_UP    38
#define PITCH_ANGLE_LIMIT_DOWN  -22
#define YAW_ANGLE_LIMIT         130


/************电机 传动比*减速比 ***************/
#define YAW_RATIO      3*19         //Yaw轴
#define Sec_YAW_RATIO  3*1          //副Yaw轴


#endif
