#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

 typedef struct SAngle
{
	short Angle[3];
	short T;
}SAngle;

void USART6_IRQHandler(void);

extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	

void Update_y(float new_y);
void Update_x(float new_x);
void uart_init(u32 bound);

void Update_yaw(float new_yaw);
void stract(char str1[],char str2[],int num);
void USART1_Init(u32 bound);
void Yaw_Init(void);
	
#endif


