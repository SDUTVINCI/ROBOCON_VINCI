#ifndef CAN_H
#define CAN_H
#include "stm32f4xx.h"
uint8_t CAN1_mode_init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode);
uint8_t CAN2_mode_init(uint8_t tsjw,uint8_t tbs2,uint8_t tbs1,uint16_t brp,uint8_t mode);
void CAN_CMD_can1_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_CMD_can1_2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_CMD_can2_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_CMD_can2_2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


typedef enum 
{ CAN_CHASSIS_ALL_ID = 0x200,
    CAN_M1_ID = 0x201,
    CAN_M2_ID = 0x202,
    CAN_M3_ID = 0x203,
    CAN_M4_ID = 0x204,
		CAN_M5_ID = 0x205,
		CAN_M6_ID = 0x206,
		CAN_M7_ID = 0x207,
		CAN_M8_ID = 0x208,	
} can_msg_id_e;

#define CHASSIS_CAN CAN1

typedef struct
{
    uint16_t angle;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    uint16_t last_angle;
		int32_t total_angle;
		int32_t	round_cnt;
		uint16_t offset_angle;
		uint32_t			msg_cnt;
} motor_measure_t;


extern motor_measure_t motor_chassis[8];

extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
void CAN1_RX0_IRQHandler(void);
void Get_Motor_Offset(motor_measure_t *ptr,CanRxMsg *rx_message);
#endif




