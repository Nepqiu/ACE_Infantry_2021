#ifndef __LED_H
#define __LED_H
#include "main.h"


//LED�˿ڶ���
#define LEDE0 PEout(0)  //ң��
#define LEDE1 PEout(1)  //����
#define LEDE2 PEout(2)	//��̨	
#define LEDE3 PEout(3)  //���	
#define LEDE4 PEout(4)	//����
#define LEDB7 PBout(7)  //����ָʾ��



void LED_Init(void);

#endif
