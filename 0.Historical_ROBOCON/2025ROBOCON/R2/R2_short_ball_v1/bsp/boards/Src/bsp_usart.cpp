#include "bsp_usart.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"

uint8_t rx_buffer[1];
remote_data receive_data;
extern int32_t Motor_target[2];;

extern "C"
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{

		Receive_data(rx_buffer[0],DATA_LEN);

 	}

}


//数据解析
void DataAnalysis(uint8_t *Data)
{
	 receive_data.ch[0]=(int16_t)(Data[1]<<8|(int16_t)Data[0]);
     receive_data.ch[1]=(int16_t)(Data[3]<<8|(int16_t)Data[2]);

	 Motor_target[0] = receive_data.ch[0];
	 Motor_target[1] = receive_data.ch[1];

}
//和校验
uint8_t SUMCRC(uint8_t *Message,uint16_t DataLen)
{
	uint16_t i=1;
	uint8_t SUM=0x00;
	for(;i<=DataLen;i++)
	{
		SUM+=Message[i];
	}

	return SUM;

}
//接收数据
void Receive_data(uint8_t Bytedata,uint8_t Data_len)
{
	static uint8_t step=0;
	static uint8_t count=0,Buff[300]={0},*data_ptr;
//	static uint16_t checksum=0;
	switch(step)
	{
		case 0:
		  if(Bytedata == 0xA5)
	  	{
  			step++;
				count=0;
				Buff[count++]=Bytedata;
				data_ptr=&Buff[count];
  		}break;
		case 1:
			Buff[count++]=Bytedata;
			if((count-1)==Data_len )
			{
				step++;
			}break;
		case 2:
			Buff[count++]=Bytedata;
			if(SUMCRC(Buff,Data_len)==Bytedata)
			{
				step++;
			}
			else
			{
				step=0;
			}break;
		case 3:
			if(Bytedata==0x5A)
			{
				DataAnalysis(data_ptr);
				step=0;
			}
			else
			{
				step=0;
			}
			break;
		default: break;
	}
}


extern "C"
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
  /* USER CODE END USART1_IRQn 1 */
}