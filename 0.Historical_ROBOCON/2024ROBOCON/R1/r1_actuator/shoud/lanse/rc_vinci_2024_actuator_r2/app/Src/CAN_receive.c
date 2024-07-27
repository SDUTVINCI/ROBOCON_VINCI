#include "can_receive.h"
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan1;
motor_measure_t motor_can2[8];
motor_measure_t motor_can1[8];
moto_info_t motor_info[MOTOR_MAX_NUM];

CAN_TxHeaderTypeDef can_tx_message;

void get_motor_measure(motor_measure_t *ptr, uint8_t data[])
{
	(ptr)->last_angle = (ptr)->angle;
	(ptr)->angle = data[0] << 8 | data[1];
	(ptr)->speed_rpm = data[2] << 8 | data[3];
	(ptr)->given_current = data[4] << 8 | data[5];
	(ptr)->temperate = data[6];
	//				((ptr)->angle) = (int32_t)(((ptr)->ecd) - ((ptr)->last_ecd));

	if (ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt--;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

void get_moto_offset(motor_measure_t *ptr, uint8_t data[])
{
	ptr->angle = data[0] << 8 | data[1];
	ptr->offset_angle = ptr->angle;
}

void get_total_angle(motor_measure_t *p)
{

	int res1, res2, delta;
	if (p->angle < p->last_angle)
	{			        // 可能的情况
		res1 = p->angle + 8192 - p->last_angle; // 正转，delta=+
		res2 = p->angle - p->last_angle;        // 反转	delta=-
	}
	else
	{			        // angle > last
		res1 = p->angle - 8192 - p->last_angle; // 反转	delta -
		res2 = p->angle - p->last_angle;        // 正转	delta +
	}
	// 不管正反转，肯定是转的角度小的那个是真的
	if (ABS(res1) < ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}

// can1   :gm3508      can2 : gm6020
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data_1[8];
	uint8_t rx_data_2[8];

	if (hcan->Instance == CAN1) // m3508
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
			// get motor id
			i = rx_header.StdId - CAN_3508_M1_ID;
			motor_can1[i].msg_cnt++ <= 50 ? get_moto_offset(&motor_can1[i], rx_data_1) : get_motor_measure(&motor_can1[i], rx_data_1);
			get_motor_measure(&motor_can1[i], rx_data_1);
			break;
		}
		default:
		{
			break;
		}
		}
	}

	if (hcan->Instance == CAN2)
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
				// get motor id
				i = rx_header.StdId - CAN_3508_M1_ID;
				motor_can2[i].msg_cnt++ <= 50 ? get_moto_offset(&motor_can2[i], rx_data_2) : get_motor_measure(&motor_can2[i], rx_data_2);
				get_motor_measure(&motor_can2[i], rx_data_2);
				break;
			}
		default:
		{
			break;
		}
		}
	}

	//	if(hcan->Instance==CAN2)//gm6020
	//	{
	//		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data_2);
	//	}
	// if ((rx_header.StdId >= FEEDBACK_ID_BASE)
	//   && (rx_header.StdId <  FEEDBACK_ID_BASE + MOTOR_MAX_NUM))                  // judge the can id
	//  {
	//
	//    uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE;                  // get motor index by can_id
	//    motor_info[index].rotor_angle    = ((rx_data_2[0] << 8) | rx_data_2[1]);
	//    motor_info[index].rotor_speed    = ((rx_data_2[2] << 8) | rx_data_2[3]);
	//    motor_info[index].torque_current = ((rx_data_2[4] << 8) | rx_data_2[5]);
	//    motor_info[index].temp           =   rx_data_2[6];
	//  }
}

void CAN1_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t send_mail_box;
	uint8_t can_send_data[8];
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

void CAN1_CMD_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
	uint32_t send_mail_box;
	uint8_t can_send_data[8];
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
void CAN2_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t send_mail_box;
	uint8_t can_send_data[8];
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

void CAN2_CMD_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
	uint32_t send_mail_box;
	uint8_t can_send_data[8];
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
