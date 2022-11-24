/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
struct SAngle 	stcAngle;

float pos_x=0;
float pos_y=0;
float xangle=0;
float yangle=0;
float zangle=0;
float w_z=0;
/* USER CODE END 0 */

UART_HandleTypeDef huart6;

/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14|GPIO_PIN_9);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
//**************陀螺仪（1）*******************//
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
 
	ucRxBuffer[ucRxCnt++]=ucData;	
	if (ucRxBuffer[0]!=0x55) //校验数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//判断一帧数据是否接收完成
	else
	{
		switch(ucRxBuffer[1])
		{
 
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;//判断是否为角度的数据
 
		}
		ucRxCnt=0;
	}
}
char wx1,wx2,wx3;
void USART6_IRQHandler(void)
{
		CopeSerial2Data((unsigned char)huart6.Instance->DR);//接收数据
		huart6.Instance->SR=0x00;
    wx1=(float)stcAngle.Angle[0]/32768*180;
		wx2=(float)stcAngle.Angle[1]/32768*180;
		wx3=(float)stcAngle.Angle[2]/32768*180;

__HAL_UART_ENABLE_IT(&huart6,UART_IT_ERR);
__HAL_UART_ENABLE_IT(&huart6,UART_IT_RXNE);
}	


//****************陀螺仪（2）*****************************//
/*获取角度值后立刻清零，不可一直保留角度的数值*/

//uint16_t USART_RX_STA=0;       //接收状态标记	 
//uint8_t  USART_RX_BUF[11];     //接收缓冲，最大USART_REC_LEN个字节
//uint16_t B,A;
//int n=0;
//uint8_t RollL,RollH,PitchL,PitchH,YawL,YawH,TL,TH,SUM;
//short wxL,wxH,wyL,wyH,wzL,wzH,Roll,Pitch,Yaw,wx,wy,wz;
//void USART6_IRQHandler(void)                	
//	{
// 	    uint8_t Res;
//		//更新中断
//	if(huart6.Instance->SR & UART_FLAG_RXNE)
// {
//  __HAL_UART_CLEAR_PEFLAG(&huart6);
//  Res = huart6.Instance->DR; 
//    {                   //取当前接收的一个字节
//		  if(n<=10)                                        //还没接收到完整的一帧数据
//                {
//                   if(Res==0x55&&n==0) USART_RX_STA|=0x8000;		//检测帧头字节，并在接收状态寄存器上（最高位置1）做下标记	 								                                      
//                   if(USART_RX_STA&0x8000)
//                   {  
//					  if(n==1&&Res==0x53) USART_RX_STA|=0x4000;     //检测是否为角度帧，并在接收状态寄存器上（高第二位置1）做下标记
//					  if(n==1&&Res==0x52) USART_RX_STA|=0x2000;		//检测是否为角速度帧，并在接收状态寄存器上（高第三位置1）做下标记							 
//                      USART_RX_BUF[n]=Res ;                         //缓存当前接收的这个字节
//		                  USART_RX_STA++ ;          //接收状态寄存器记录此帧数据的接收进程
//		                  n=USART_RX_STA&0x1FFF ;                   //转接进程记录
//                    }										
//					if(n==2) 
//					{
//					    if(((USART_RX_STA&0xf000)!=0xc000)&&((USART_RX_STA&0xf000)!=0xa000))  //检测是否成功接收帧前两字节
//						{
//                          USART_RX_STA=0 ;                           //接收状态寄存器清零
//		                  n=USART_RX_STA&0x1FFF;                     //重新开始接收
//                        } 
//					}
//                 }          
//                   if(n==11)                                         //成功接收到一帧数据
//                   {       
//                      if((USART_RX_STA&0xf000)==0xc000)              //检测是否为角度帧
//                      {
//                         RollL=USART_RX_BUF[2];
//                         RollH=USART_RX_BUF[3];
//                         PitchL=USART_RX_BUF[4];
//                         PitchH=USART_RX_BUF[5];
//                         YawL=USART_RX_BUF[6];
//                         YawH=USART_RX_BUF[7];
//                         TL=USART_RX_BUF[8];
//                         TH=USART_RX_BUF[9];
//                         SUM=USART_RX_BUF[10];
//                         if(SUM==(0x55+0x53+RollH+RollL+PitchH+PitchL+YawH+YawL+TH+TL)%0x100) //和校验，注意这里去除了高八位上的和
//                         { 
//                            Roll=(short)((RollH<<8)|RollL)/32768.0*2000;
//                            Pitch=(short)((PitchH<<8)|PitchL)/32768.0*2000;
//                            Yaw=(short)((YawH<<8)|YawL)/32768.0 *2000;
//		                        n=0;	              //数据处理完成
//													  B=USART_RX_STA;   
//                            USART_RX_STA=0;			  //清零，待重新接收										 
//                          }
//                         else
//                           {
//                             USART_RX_STA=0 ;
//		                         n=0; 
//                            }                  
//                       }
//                       if((USART_RX_STA&0xf000)==0xa000)              //检测是否为角速度帧
//                      {
//                         wxL=USART_RX_BUF[2]; 
//                         wxH=USART_RX_BUF[3];
//                         wyL=USART_RX_BUF[4];
//                         wyH=USART_RX_BUF[5];
//                         wzL=USART_RX_BUF[6];
//                         wzH=USART_RX_BUF[7];
//                         TL=USART_RX_BUF[8];
//                         TH=USART_RX_BUF[9];
//                         SUM=USART_RX_BUF[10];
//                         if(SUM==(0x55+0x52+wxH+wxL+wyH+wyL+wzH+wzL+TH+TL)%0x100) //和校验，注意这里去除了高八位上的和
//                         {
//                            wx=(short)((wxH<<8)|wxL)/32768.0*2000;
//                            wy=(short)((wyH<<8)|wyL)/32768.0*2000;
//                            wz=(short)((wzH<<8)|wzL)/32768.0*2000;                           
//		                        n=0;                 //数据处理完成
//													  A=USART_RX_STA;
//													  USART_RX_STA=0;  	//清零，待重新接收									 
//                          }
//                         else         //接收错误
//                           {
//                             USART_RX_STA=0 ;
//		                         n=0; 
//                            }         //清零，待重新接收         
//                       } 
//                    }	
//                    if(n>=11)         //数据处理结束
//                     {
//                       USART_RX_STA=0 ;
//		                   n=0; 
//                    }                 //清零，待重新接收
//              if(A==0xa00b&&B==0xc00b) {
//__HAL_UART_ENABLE_IT(&huart6,UART_IT_ERR);
//__HAL_UART_ENABLE_IT(&huart6,UART_IT_RXNE);		
//	 } 

// }
//	}
//	
//}
/* USER CODE END 1 */
