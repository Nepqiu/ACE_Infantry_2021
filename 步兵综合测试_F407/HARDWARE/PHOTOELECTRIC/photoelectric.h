#ifndef __PHOTOELECTRIC_H
#define __PHOTOELECTRIC_H
#include "main.h"

void Photoelectric_GPIO_Init(void);
void Yaw_Zero_GPIO_Init(void);     //���У׼������
void Yaw_Two_Zero_GPIO_Init(void);

u8 Yaw_Zero_Value(void);
u8 Sec_Yaw_Zero_Value(void);   //���Y���Ƿ�λ   ����ֵ����0�����򷵻�1

#endif
