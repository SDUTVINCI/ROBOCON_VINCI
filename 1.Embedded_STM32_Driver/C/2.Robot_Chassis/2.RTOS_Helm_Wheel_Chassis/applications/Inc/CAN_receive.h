/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef __CAN_RECEIVE_H
#define __CAN_RECEIVE_H

#include "include.h"


/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
    CAN_3508_M5_ID = 0x205,
    CAN_3508_M6_ID = 0x206,
    CAN_3508_M7_ID = 0x207,
    CAN_3508_ALL_ID = 0x1FF,
} can_msg_id_e;



typedef struct
{
 uint16_t angle;
 int16_t speed_rpm;
 int16_t given_current;
 uint8_t temperate;
 int16_t last_angle;
		int32_t total_angle;
		int32_t	round_cnt;
		uint16_t offset_angle;
		uint32_t			msg_cnt;
} motor_measure_t;



void get_motor_measure(motor_measure_t *ptr,uint8_t data[]);
void get_moto_offset(motor_measure_t *ptr, uint8_t data[]);
void get_total_angle(motor_measure_t *p);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

void CAN1_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN1_CMD_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
void CAN2_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN2_CMD_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);

#endif
