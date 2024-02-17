#ifndef __CAN_USER__
#define __CAN_USER__

#include <stdio.h>
#include <stdlib.h>
#include "main.h"

void CAN1_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN2_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void filter1_init(void);
void filter2_init(void);

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
		uint32_t	msg_cnt;
}motor_measure_t;

typedef struct{
		float motor_speed_pid[3];
		float motor_position_pid[3];
}PID_Data;



void get_moto_offset(motor_measure_t *ptr, uint8_t data[]);
void get_total_angle(motor_measure_t *p);
void get_motor_measure(motor_measure_t *ptr,uint8_t data[]);                                                     

void pid_chassis_init(void);
float PID_velocity_realize_1(float set_speed,int i);
float PID_position_realize_1(float set_pos,int i);
float pid_call_1(float position,int i);
float PID_velocity_realize_2(float set_speed,int i);
float PID_position_realize_2(float set_pos,int i);
float pid_call_2(float position,int i);



#endif






