#ifndef __USART_H
#define __USART_H
#include "main.h"

//volatile
#define USART1_RX_LEN  18
#define USART1_TX_LEN  18
//extern uint8_t Usart1_Rx[USART1_RX_LEN];	
void Usart1_Init(uint8_t *rx1_buf, uint16_t dma_buf_num);

#define USART2_RX_LEN  25
#define USART2_TX_LEN  25
extern u8 Usart2_Rx[USART2_RX_LEN];
extern u8 Usart2_Tx[USART2_TX_LEN];
void Usart2_Init(void);

#endif


