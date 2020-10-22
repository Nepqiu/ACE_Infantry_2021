#ifndef __CAN_H
#define __CAN_H
#include "main.h"


//╣вел
//#define  CANx                 CAN1
//#define  CAN_TX_GPIO_PROT     GPIOA
//#define  CAN_TX_GPIO_PIN      GPIO_Pin_12
//#define  CAN_RX_GPIO_PORT     GPIOA
//#define  CAN_RX_GPIO_PIN      GPIO_Pin_11

uint8_t CAN1_mode_init(uint8_t tsjw, uint8_t tbs2, uint8_t tbs1, uint16_t brp, uint8_t mode);
uint8_t CAN2_mode_init(uint8_t tsjw, uint8_t tbs2, uint8_t tbs1, uint16_t brp, uint8_t mode);

#endif /* __BSP_CAN_H */

																					
																					
																					
																					
																					
																					
																					
																					
																					
																					
																					
																					
																					
