#include "bsp_usart.h"
#include "Communition.h"
#include "DR16.h"
#include "chassic.h"
#include "my_task.h"
#include "usart.h"
#include <cstring>
#include <stdint.h>



remote_data receive_data;
extern Rxbuffer all_rxbuff;

uint8_t stop;

int16_t N5065_traget_speed[2];
uint8_t i = 0;
extern "C"
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART7)
	{
		Receive_data(all_rxbuff.Buletooth,DATA_LEN);
		
 	}
	// if (huart->Instance == UART5) {
	// 		HAL_UART_Receive_IT(&huart5, &all_rxbuff.DR16[i++], 1);
	// 		if (i==18) {
	// 			i=0;
	// 			Get_Remote_Date();
	// 		}
	// }

}
extern "C"
void UART7_IRQHandler(void)
{
	/* USER CODE BEGIN UART7_IRQn 0 */

	/* USER CODE END UART7_IRQn 0 */
		HAL_UART_IRQHandler(&huart7);
	/* USER CODE BEGIN UART7_IRQn 1 */
		HAL_UART_Receive_IT(&huart7, &all_rxbuff.Buletooth, 1);

  /* USER CODE END UART7_IRQn 1 */
}
	uint8_t data_buf[7];
void BlueTooth_Send_Data( uint8_t data_len)
{

	data_buf[0] = 0xA5;
	data_buf[6] = 0x5A;
	
	// for (uint8_t i = 1 ; i <= data_len; i++) {
	// 	data_buf[i] = data[i];
	// }
	memcpy(&data_buf[1], &robot.upper_data[3], 4);
	data_buf[5] = SUMCRC(data_buf, data_len);

	HAL_UART_Transmit(&huart7, data_buf, data_len+3, 100);
}

//数据解析
void DataAnalysis(uint8_t *Data)
{
	robot.stop = Data[0];
	robot.rod_state.bottom = Data[1];
	receive_data.ch[0]=(int16_t)(Data[4]<<8|(int16_t)Data[3]);
    receive_data.ch[1]=(int16_t)(Data[6]<<8|(int16_t)Data[5]);
	
	
// if (all_rxbuff.s2 == 2) {
	robot.BT_target_angle = Data[2];
//	N5065_traget_speed[0] = N5065_traget_speed[1] = receive_data.ch[0];

	
	// N5065_traget_speed[1] = receive_data.ch[1];
}
// }
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

