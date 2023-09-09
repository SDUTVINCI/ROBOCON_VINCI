#ifndef _MY_UART_
#define _MY_UART_

#include "include.h"

typedef struct
{
	fp32 pos_x;
	fp32 pos_y;
	fp32 zangle;
	fp32 xangle;
	fp32 yangle;
	fp32 w_z;
} encoding_disk_measure_t;

void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);

uint16_t USART_ReceiveData(USART_TypeDef* USARTx);

void Data_Analyse(uint8_t rec);



void stract(char str1[],char str2[],int num);
void Update_x(float new_x);
void Update_y(float new_y);
void Update_yaw(float new_yaw);

#endif
