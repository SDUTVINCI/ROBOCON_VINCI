#include "IMU.h"

#include "usart.h"
#include "postion_clac.h"
#include "pid.h"
#include "CAN_PID_User.h"
#include "wit_c_sdk.h"



extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;


float roll,pitch,yaw;
uint16_t sad,jas,isa;
uint8_t fasa;
uint8_t ucRxBuffer[132];
float roll,pitch,yaw;
uint16_t sad,jas,isa;
uint8_t fasa;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static float yaw_star,roll_star,pitch_star;
  if(huart==&huart1)
  {
	for(uint8_t i=0;i<250;i++){
			if (ucRxBuffer[i]==0x55){ //数据头不对，则重新开始寻找0x55数据头
				if(ucRxBuffer[i+1]==0x53){

		sad=(uint16_t)ucRxBuffer[i+3]<<8|(uint16_t)ucRxBuffer[i+2];
		roll=sad/32768.0f*180.0f;
		jas=(uint16_t)ucRxBuffer[i+5]<<8|(uint16_t)ucRxBuffer[i+4];
		pitch=jas/32768.0f*180.0f;
		isa=(uint16_t)ucRxBuffer[i+7]<<8|(uint16_t)ucRxBuffer[i+6];
		yaw=isa/32768.0f*180.0f;
			break;
			}
				continue;

			}

  }
			if(fasa==0){
				yaw_star=yaw;
				roll_star=roll;
				pitch_star=pitch;
				fasa=1;
				
			}
			yaw=yaw-yaw_star;
			roll=roll-roll_star;
			pitch=pitch-pitch_star;
//	HAL_UART_Receive_IT(&huart1,ucRxBuffer,132);
	HAL_UART_Receive_DMA(&huart1,ucRxBuffer,132);


		}

	}


//uint8_t rx_cnt;
//uint8_t ucRxBuffer[132];
//float roll,pitch,yaw;
//uint16_t sad,jas,isa;
//uint8_t fasa;

//void IMU_Data_Clac(uint8_t  data[])
//{
//	static float yaw_star;

//	for(uint8_t i=0;i<250;i++){
//			if (data[i]==0x55){ //数据头不对，则重新开始寻找0x55数据头
//				if(data[i+1]==0x53){

//		sad=(uint16_t) data[i+3]<<8|data[i+2];
//		roll=sad/32768*180;
//		jas=(uint16_t)data[i+5]<<8|(uint16_t)data[i+4];
//		pitch=jas/32768*180;
//		isa=(uint16_t)data[i+7]<<8|(uint16_t)data[i+6];
//		yaw=isa/32768*180;
//			break;
//			}
//				continue;

//			}

//  }
//			if(fasa==0){
//				yaw_star=yaw;
//				fasa=1;
//			}
//			yaw=yaw-yaw_star;



//		}




//void USART1_IRQHandler(void){
//	
//          IMU_Read_Euler(ucRxBuffer);

//		     HAL_UART_Receive_DMA(&huart1,ucRxBuffer,132);//开启DMA接收，方便下一次接收数据
//}


//uint8_t __SUMCRC(uint8_t *puchMsg, uint16_t usDataLen)
//{
//    int16_t i = 0;
//		uint8_t uchSUMCRC = 0x00;
//    for (; i < usDataLen; i++)
//    {
//			uchSUMCRC += puchMsg[i];
//    }
//    return uchSUMCRC;
//}


//bool_t IMU_Read_Euler(uint8_t * Data)
//{
//	static uint16_t SUMCRC;
//	uint8_t rx_buffer_pre;
//	for(int i = 0;i < 132;i++)
//	{
//		if(rx_buffer_pre == 0x55 && ucRxBuffer[i] == 0x53)
//		{
//			rx_buffer_pre = ucRxBuffer[i];
//			rx_cnt = i - 1;
//		}
//	}
//	
//	if(ucRxBuffer[rx_cnt] != 0x55)
//	{
//		return 0;
//	}
//	if(ucRxBuffer[rx_cnt + 1] != 0x53)
//	{
//		return 0;
//	}
//	else
//	{
//		SUMCRC = __SUMCRC(&(ucRxBuffer[rx_cnt]),10);
//		if(SUMCRC != ucRxBuffer[rx_cnt + 10])
//		{
//			return 0;
//		}
//		roll = (int16_t)((int16_t)ucRxBuffer[rx_cnt + 3] << 8 | ucRxBuffer[rx_cnt + 2]);
//		pitch = (int16_t)((int16_t)ucRxBuffer[rx_cnt + 5] << 8 | ucRxBuffer[rx_cnt + 4]);
//		yaw = (int16_t)((int16_t)ucRxBuffer[rx_cnt + 7] << 8 | ucRxBuffer[rx_cnt + 6]);

//		roll = roll / 32768.0f * 180.0f;
//		pitch = pitch / 32768.0f * 180.0f;
//		yaw = yaw / 32768.0f * 180.0f;
//		
//		return 1;
//	}
//}


