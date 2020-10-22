#ifndef __MAIN_H
#define __MAIN_H

/*
Y:v77r7r7r7r7r7r7rriri:rgQMMRZDZRddDggZDRbgDDbDgQU
v.::::::::::::::::.:.. ibEXdqKSXqK5qXbUPXSSK55PdE:
7:777rrrrr7rrrrrrrrir::1BgDggdDEggZdggDQEdgPZdBgB 
7:rrrriririririiiriri::qBggMZDZgZDPZgZZQbgMEdD1BZ 
7:rrriiiiiiiiii:iiiii.:PBZMgZZZDgEddRPQgdgMEQJ:BP 
r:rriiiriiii:iii:i:::.:EQDggdDERDZbggdQgbBgZQ7rBqi
r.iii:i:i:::::::i::::.:ZBZMgZdZRgEEgMdBgB2ZBg .B: 
i.:i:::i:::::::::::::.:gQDggZZbQMDPQDQPRK QB7 :B: 
i.i:i::::::::::::::.. idQEMggEEgQZdQMQrB. QP. iQ7:
:.::.:::::::::::.:.:...qQDgMEDERRgPQBu.d .7:   q. 
: :::::.:.:...:.:.... .2QDRgDZZgQZdQB :QbQRQBBQD. 
: ::.:.:.:.....:..... .EZRgREDdDQgdBq.BBXBBBQ:ZBB.
: :.:.:...:.......... rjLBgRDEZdBgZB7.Q: 5BQZ1BB7r
: ::.:...:..... ....  v7.BMQDDEdQgZBr  ...r:..7r  
: :.:.:.:.....vR75Pvu:rv UBRgEDdQMMQj ..:...... . 
:.::.:.:..:.riiE:J: BYU77:ZQMZEEDQBsr..:::.:.:....
:.:::.:..:L7U.  .5    L:  URREDdDQBJi:::::::::::: 
:.::::::::::ri           5KPMgEZdQQ5.i.:::::::.:: 
i.:::::::::::.:::..   rMBRDdMgZEdDBP.i:.:::::...: 
i.::::::::::::::::.JZBBBQQDMZMEDbZRQ.:.:.:.:.:..: 
i.i::::::::::::::.7BMQBBgdMDDZgEEbQQ..:.........: 
:.::::::::.::::irr7r.iQBQgbgZEgEdEZB: ........... 
i.i:::::::rr:   iY7ii:sZQMEbDPQZPEZBs ........... 
i.i::i::::iuL:   :v7iiirMggbbbDQqdbQg. .......... 
r.i:i:...  .sY:   :7irj.sMQgPPEBZXdEQr:. ........ 
r:ir:  . .  .YY:   ir7I.:QdBDPPBQqbdQ7i7.  ...... 
7.rr.......: .sv.   ivX:.XgqgPDgRKdqQv:r7i.  .... 
r:7: ..... :. :Jv.   rEi.sMbqPdDMddED1.rir7:.. .. 
7 :             i:    vr .P1jUuKdrYKS7 .....      
*/

//系统头文件
#include "stm32f4xx.h" 
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

//FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

//SYSTEM
#include "SysInit.h"//初始化头文件
#include "sys.h"
#include "delay.h"

//HARDWARE
//#include "led.h"
//#include "key.h"
//#include "can.h"
//#include "pid.h"
//#include "time.h"
//#include "iwdg.h"
//#include "usart.h"
//#include "photoelectric.h"

//REFEREE

//DEFINE
#include "Mode_Def.h"
#include "PID_Def.h"
#include "Speed_Def.h"

//CONTROL
#include "maths.h"
#include "filter.h"
//#include "RemoteControl.h"
//#include "CAN_receive.h"
//#include "chassis_behaviour.h"
//#include "IMU.h"
//#include "rmmotor.h"

//Task
#include "Task_Start.h"
//#include "Task_Remote.h"
//#include "Task_Detect.h"
//#include "Task_Chassis.h"
//#include "Task_Gimbal.h"
//#include "Task_Safecheck.h"
//#include "Task_Test.h"




#define RC_NVIC   0  //遥控器   
#define CAN1_NVIC 4
#define CAN2_NVIC 4
#define TIM3_NVIC 5
#define TIM6_NVIC 4
#define SPI5_RX_NVIC 5
#define MPU_INT_NVIC 5


#endif
