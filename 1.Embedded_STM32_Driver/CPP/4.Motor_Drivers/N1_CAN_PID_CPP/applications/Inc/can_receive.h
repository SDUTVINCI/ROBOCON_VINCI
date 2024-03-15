#ifndef __CAN_RECEIVE_H_
#define __CAN_RECEIVE_H_

#ifdef __cplusplus
extern "C" 
{
#endif

#include "startup_main.h"
#include "can.h"
	
#define ABS(x)	( (x>0) ? (x) : (-x) )

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
 uint8_t temperature;
 int16_t last_angle;
		int32_t total_angle;
		int32_t	round_cnt;
		uint16_t offset_angle;
		uint32_t			msg_cnt;
} motor_measure_t;



class CAN_BUS
{
	public:
		motor_measure_t motor_can1[8];
		motor_measure_t motor_can2[8];
		
		class BSP
		{
			public:
				void CAN_Start(CAN_HandleTypeDef *hcan);
				void Filter_Init(CAN_HandleTypeDef *hcan);
		}bsp;
		
		class DJI_ENCODER
		{
			public:
				void get_motor_measure(motor_measure_t *ptr,uint8_t data[]);
				void get_moto_offset(motor_measure_t *ptr, uint8_t data[]);
				void get_total_angle(motor_measure_t *p);
		}dji_encoder;
		
		class CMD
		{
			public:
				void CAN1_Front(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
				void CAN1_Behind(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
				void CAN2_Front(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
				void CAN2_Behind(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
		}cmd;
};
/*******对象*******/	
extern CAN_BUS can_bus;

#ifdef __cplusplus
}
#endif

#endif
