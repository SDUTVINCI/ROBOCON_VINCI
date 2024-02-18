#include "can_receive.h"
#include "can.h"
/*******对象*******/
CAN_BUS can_bus;


/**
 * @brief       CAN1发送函数(前4个电机)
 * @param       motor1：给数组序号为0（电调ID为1）的电机发送相对电流值
 * @param       motor2：给数组序号为1（电调ID为2）的电机发送相对电流值
 * @param       motor3：给数组序号为2（电调ID为3）的电机发送相对电流值
 * @param       motor4：给数组序号为3（电调ID为4）的电机发送相对电流值
 * @retval      void
 * @note        无
 */
void CAN_BUS::CMD::CAN1_Front(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
 CAN_TxHeaderTypeDef can_tx_message;
 uint8_t	can_send_data[8];
 uint32_t send_mail_box;
	
 can_tx_message.StdId = 0x200;
 can_tx_message.IDE = CAN_ID_STD;
 can_tx_message.RTR = CAN_RTR_DATA;
 can_tx_message.DLC = 0x08;
 can_send_data[0] = motor1 >> 8;
 can_send_data[1] = motor1;
 can_send_data[2] = motor2 >> 8;
 can_send_data[3] = motor2;
 can_send_data[4] = motor3 >> 8;
 can_send_data[5] = motor3;
 can_send_data[6] = motor4 >> 8;
 can_send_data[7] = motor4;
 HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
}



/**
 * @brief       CAN1发送函数(后4个电机)
 * @param       motor1：给数组序号为4（电调ID为5）的电机发送相对电流值
 * @param       motor2：给数组序号为5（电调ID为6）的电机发送相对电流值
 * @param       motor3：给数组序号为6（电调ID为7）的电机发送相对电流值
 * @param       motor4：给数组序号为7（电调ID为8）的电机发送相对电流值
 * @retval      void
 * @note        无
 */
 void CAN_BUS::CMD::CAN1_Behind(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
 uint8_t	can_send_data[8];
 uint32_t send_mail_box;
 CAN_TxHeaderTypeDef can_tx_message;
	
 can_tx_message.StdId = 0x1FF;
 can_tx_message.IDE = CAN_ID_STD;
 can_tx_message.RTR = CAN_RTR_DATA;
 can_tx_message.DLC = 0x08;
 can_send_data[0] = motor5 >> 8;
 can_send_data[1] = motor5;
 can_send_data[2] = motor6 >> 8;
 can_send_data[3] = motor6;
 can_send_data[4] = motor7 >> 8;
 can_send_data[5] = motor7;
 can_send_data[6] = motor8 >> 8;
 can_send_data[7] = motor8;
 HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
}


/**
 * @brief       CAN2发送函数(前4个电机)
 * @param       motor1：给数组序号为0（电调ID为1）的电机发送相对电流值
 * @param       motor2：给数组序号为1（电调ID为2）的电机发送相对电流值
 * @param       motor3：给数组序号为2（电调ID为3）的电机发送相对电流值
 * @param       motor4：给数组序号为3（电调ID为4）的电机发送相对电流值
 * @retval      void
 * @note        无
 */
 void CAN_BUS::CMD::CAN2_Front(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
 CAN_TxHeaderTypeDef can_tx_message;
 uint8_t	can_send_data[8];
 uint32_t send_mail_box;
	
 can_tx_message.StdId = 0x200;
 can_tx_message.IDE = CAN_ID_STD;
 can_tx_message.RTR = CAN_RTR_DATA;
 can_tx_message.DLC = 0x08;
 can_send_data[0] = motor1 >> 8;
 can_send_data[1] = motor1;
 can_send_data[2] = motor2 >> 8;
 can_send_data[3] = motor2;
 can_send_data[4] = motor3 >> 8;
 can_send_data[5] = motor3;
 can_send_data[6] = motor4 >> 8;
 can_send_data[7] = motor4;
 HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, can_send_data, &send_mail_box);
}

/**
 * @brief       CAN1发送函数(后4个电机)
 * @param       motor1：给数组序号为4（电调ID为5）的电机发送相对电流值
 * @param       motor2：给数组序号为5（电调ID为6）的电机发送相对电流值
 * @param       motor3：给数组序号为6（电调ID为7）的电机发送相对电流值
 * @param       motor4：给数组序号为7（电调ID为8）的电机发送相对电流值
 * @retval      void
 * @note        无
 */
 void CAN_BUS::CMD::CAN2_Behind(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
 CAN_TxHeaderTypeDef can_tx_message;
 uint8_t	can_send_data[8];
 uint32_t send_mail_box;

 can_tx_message.StdId = 0x1FF;
 can_tx_message.IDE = CAN_ID_STD;
 can_tx_message.RTR = CAN_RTR_DATA;
 can_tx_message.DLC = 0x08;
 can_send_data[0] = motor5 >> 8;
 can_send_data[1] = motor5;
 can_send_data[2] = motor6 >> 8;
 can_send_data[3] = motor6;
 can_send_data[4] = motor7 >> 8;
 can_send_data[5] = motor7;
 can_send_data[6] = motor8 >> 8;
 can_send_data[7] = motor8;
 HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, can_send_data, &send_mail_box);
}

extern "C"
/**
 * @brief       回调函数，定时器中断服务函数调用
 * @param       hcan：触发中断的CAN通信句柄
 * @retval      void
 * @note        该函数由中断公共服务函数调用，不用用户去调用。且为一个弱函数，所以在C++中要在该函数前面加上extern "C"，或直接用extern "C"{}括起来
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		CAN_RxHeaderTypeDef rx_header;
		uint8_t rx_data_1[8];
		uint8_t rx_data_2[8];
	
	if(hcan->Instance==CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data_1);

		switch (rx_header.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			case CAN_3508_M5_ID:
			case CAN_3508_M6_ID:
			case CAN_3508_M7_ID:
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_3508_M1_ID;
				can_bus.motor_can1[i].msg_cnt++ <= 50	?	can_bus.dji_encoder.get_moto_offset(&can_bus.motor_can1[i], rx_data_1) : can_bus.dji_encoder.get_motor_measure(&can_bus.motor_can1[i], rx_data_1);
				can_bus.dji_encoder.get_motor_measure(&can_bus.motor_can1[i], rx_data_1);
				break;
			}
			default:
			{
			break;
			}	
		}
	}
		
	if(hcan->Instance==CAN2)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data_2);

		switch (rx_header.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			case CAN_3508_M5_ID:
			case CAN_3508_M6_ID:
			case CAN_3508_M7_ID:
//			case CAN_3508_ALL_ID:
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_3508_M1_ID;
				can_bus.motor_can2[i].msg_cnt++ <= 50	?	can_bus.dji_encoder.get_moto_offset(&can_bus.motor_can2[i], rx_data_2) : can_bus.dji_encoder.get_motor_measure(&can_bus.motor_can2[i], rx_data_2);
				can_bus.dji_encoder.get_motor_measure(&can_bus.motor_can2[i], rx_data_2);
				break;
			}
			default:
			{
			break;
			}	
		}
 }
}

/**
 * @brief       处理编码器数据获取电机信息函数
 * @param       ptr：@motor_measure_t类型句柄，储存处理后的电机状态信息
 * @param       data：CAN通信接收到电机发来的原数据
 * @retval      void
 * @note        无
 */
void CAN_BUS::DJI_ENCODER::get_motor_measure(motor_measure_t *ptr,uint8_t data[])                                                     
{   
        (ptr)->last_angle = (ptr)->angle;                                                          
        (ptr)->angle = data[0] << 8 | data[1];           
        (ptr)->speed_rpm = data[2] << 8 | data[3];     
        (ptr)->given_current = data[4] << 8 | data[5]; 
        (ptr)->temperate = data[6];                                              
//				((ptr)->angle) = (int32_t)(((ptr)->ecd) - ((ptr)->last_ecd));

					if(ptr->angle - ptr->last_angle > 4096)
						ptr->round_cnt --;
					else if (ptr->angle - ptr->last_angle < -4096)
						ptr->round_cnt ++;
					ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

/**
 * @brief       处理编码器数据获取电机刚开始的角度初始值函数
 * @param       ptr：@motor_measure_t类型句柄，储存处理后的电机状态信息
 * @param       data：CAN通信接收到电机发来的原数据
 * @retval      void
 * @note        该函数是将绝对编码器转化为增量编码器的关键，获取了电机通电时的角度值
 */
void CAN_BUS::DJI_ENCODER::get_moto_offset(motor_measure_t *ptr, uint8_t data[])
{
	ptr->angle = data[0]<<8 |data[1] ;
	ptr->offset_angle = ptr->angle;
}





/**
 * @brief       处理电机角度值
 * @param       p：@motor_measure_t类型句柄，储存处理后的电机状态信息
 * @retval      void
 * @note        该函数没有被调用，暂时没用
 */
void CAN_BUS::DJI_ENCODER::get_total_angle(motor_measure_t *p)
{
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//可能的情况
		res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
		res2 = p->angle - p->last_angle;				//反转	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
		res2 = p->angle - p->last_angle;				//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}


