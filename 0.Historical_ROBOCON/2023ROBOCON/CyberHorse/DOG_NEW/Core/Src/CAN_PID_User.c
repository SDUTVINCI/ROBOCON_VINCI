#include "CAN_PID_User.h"
#include "can.h"
#include "pid.h"



#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
CAN_TxHeaderTypeDef can_tx_message;
		
#define CAN_3508_M1_ID 0x201
#define CAN_3508_M2_ID 0x202
#define CAN_3508_M3_ID 0x203
#define CAN_3508_M4_ID 0x204

motor_measure_t motor_can1[8];
motor_measure_t motor_can2[8];

//PID_Data CAN_Pid_Data[]={
//{{0	.65,0, 0.2},{8.45, 0, 0.1}},
//{{0.20,0, 0.8},{12,0,0.5}},

//};
uint16_t pid_data_use;
PID_Data CAN1_PID_DATA[][4]={
// 0  walk
{
{{8,0.01, 0.02 },{0.7,0.01, 0.095}},{{8, 0.01, 0.02},{0.7,0.01,0.095}},
{{8,0.01, 0.02},{0.7,0.01, 0.095}},{{8, 0.01, 0.02},{0.7,0.01,0.095}},
},

	// 1  trot
{
{{8,0.005, 0.4 },{1.14,0, 0.2}},{{8, 0.005, 0.4},{1.14,0,0.2}},
	{{8.0, 0.005, 0.4},{1.14,0, 0.2}},{{8, 0.005, 0.4},{1.14,0,0.2}}
},

//  2 jump star
{
{{8,0.005, 0.4 },{1.0,0, 0.2}},{{8, 0.005, 0.4},{1.0,0,0.2}},
	{{8.0, 0.005, 0.4},{1.0,0, 0.2}},{{8, 0.005, 0.4},{1.0,0,0.2}}
},
		// 3  jump
{
{{400.0, 0, 0.5},{400.0,0.0,0.5}},{{400.0, 0, 0.5},{400.0,0.0,0.5}},
{{400.0, 0, 0.5},{400.0,0.0,0.5}},{{400.0, 0, 0.5},{400.0,0.0,0.5}},
},

//  4  jump_stop 1
{
{{3.0, 0, 0.2},{1.14,0,0.2}},{{3.0, 0, 0.05},{1.14,0,0.2}},
	{{3.0, 0, 0.2},{1.14,0,0.2}},{{3.0, 0, 0.05},{1.14,0,0.2}}
},

//  5  jump_stop2
{
{{3.0, 0, 0.2},{0.1,0, 0.1}},{{3.0, 0, 0.05},{0.1,0,0.1}},
	{{3.0, 0, 0.2},{0.1,0, 0.1}},{{3.0, 0, 0.05},{0.1,0,0.1}}
},

//  6  stop
{
{{8,0.01, 0.02 },{0.8,0.01, 0.095}},{{8, 0.01, 0.02},{0.8,0.01,0.095}},
{{8,0.01, 0.02},{0.8,0.01, 0.095}},{{8, 0.01, 0.02},{0.8,0.01,0.095}},
},
//   7  jump_ceshi

{
{{700.0, 0, 0.5},{700.0,0.0,0.5}},{{700.0, 0, 0.5},{700.0,0.0,0.5}},
{{600.0, 0, 0.5},{600.0,0.0,0.5}},{{600.0, 0, 0.5},{600.0,0.0,0.5}},
},

// 8 jump end1
{
{{15.0,0.005, 0.4 },{1.80,0, 0.2}},{{15.0, 0.005, 0.4},{1.80,0,0.2}},
	{{15.0, 0.005, 0.4},{1.80,0, 0.2}},{{15.0, 0.005, 0.4},{1.80,0,0.2}}
},


};



PID_Data CAN2_PID_DATA[][4]={
// 0  walk
{
{{8,0.01, 0.02 },{0.7,0.01, 0.095}},{{8, 0.01, 0.02},{0.7,0.01,0.095}},
{{8,0.01, 0.02},{0.7,0.01, 0.095}},{{8, 0.01, 0.02},{0.7,0.01,0.095}},
},
	// 1  trot
{
{{8,0.005, 0.4 },{1.14,0, 0.2}},{{8, 0.005, 0.4},{1.14,0,0.2}},
	{{8.0, 0.005, 0.4},{1.14,0, 0.2}},{{8, 0.005, 0.4},{1.14,0,0.2}}
},

//  2 jump star
{
{{8,0.005, 0.4 },{1.0,0, 0.2}},{{8, 0.005, 0.4},{1.0,0,0.2}},
	{{8.0, 0.005, 0.4},{1.0,0, 0.2}},{{8, 0.005, 0.4},{1.0,0,0.2}}
},
		// 3  jump
{
{{400.0, 0, 0.5},{400.0,0.0,0.5}},{{400.0, 0, 0.5},{400.0,0.0,0.5}},
{{340.0, 0, 0.5},{340.0,0.0,0.5}},{{340.0, 0, 0.5},{340.0,0.0,0.5}},
},

//  4  jump_stop 1
{
{{3.0, 0, 0.2},{1.14,0,0.2}},{{3.0, 0, 0.05},{1.14,0,0.2}},
	{{3.0, 0, 0.2},{1.14,0,0.2}},{{3.0, 0, 0.05},{1.14,0,0.2}}
},

//  5  jump_stop2
{
{{3.0, 0, 0.2},{0.1,0, 0.1}},{{3.0, 0, 0.05},{0.1,0,0.1}},
	{{3.0, 0, 0.2},{0.1,0, 0.1}},{{3.0, 0, 0.05},{0.1,0,0.1}}
},

//  6  stop
{
{{8,0.01, 0.02 },{0.8,0.01, 0.095}},{{8, 0.01, 0.02},{0.8,0.01,0.095}},
{{8,0.01, 0.02},{0.8,0.01, 0.095}},{{8, 0.01, 0.02},{0.8,0.01,0.095}},
},
//   7  jump_ceshi

{
{{700.0, 0, 0.5},{700.0,0.0,0.5}},{{700.0, 0, 0.5},{700.0,0.0,0.5}},
{{600.0, 0, 0.5},{600.0,0.0,0.5}},{{600.0, 0, 0.5},{600.0,0.0,0.5}},
},

// 8 jump end1
{
{{15.0,0.005, 0.4 },{1.80,0, 0.2}},{{15.0, 0.005, 0.4},{1.80,0,0.2}},
	{{15.0, 0.005, 0.4},{1.80,0, 0.2}},{{15.0, 0.005, 0.4},{1.80,0,0.2}}
},

};

float IMU_PID[3]={1.0f,0,0};

//float PID_FORCE[3]={2.0f,0,0};


int min_output,max_i_output,angle_min_output,angle_i_max_output;
double angle_motor;
uint8_t X_H;
pid_type_def pid_v1[8],pid_pos1[8];
pid_type_def pid_v2[8],pid_pos2[8];
pid_type_def pid_imu;
pid_type_def pid_force[8];

float motor_position_3508_pid[3] = {0.35,0, 0.0};
float motor_speed_3508_pid[3] = {9.6, 0, 0.1};



void pid_chassis_init(void)
{
	for(int i=0;i<4;i++)
	{

		PID_init(&pid_v1[i], PID_POSITION,CAN1_PID_DATA[pid_data_use][i].motor_speed_pid, min_output,max_i_output);
		PID_init(&pid_pos1[i], PID_POSITION, CAN1_PID_DATA[pid_data_use][i].motor_position_pid, angle_min_output,angle_i_max_output);
//		PID_init(&pid_force[i], PID_POSITION,PID_FORCE , exp_A[0],exp_A[1]);
	}
	
		for(int i=0;i<4;i++)  
	{

		PID_init(&pid_v2[i], PID_POSITION, CAN2_PID_DATA[pid_data_use][i].motor_speed_pid,  min_output,max_i_output);
		PID_init(&pid_pos2[i], PID_POSITION, CAN2_PID_DATA[pid_data_use][i].motor_position_pid, angle_min_output,angle_i_max_output);
	}
		PID_init(&pid_imu, PID_POSITION,IMU_PID , 300.0f,0.0f);


}
         /***********can1电机PID***************/
float PID_velocity_realize_1(float set_speed,int i)
{
		PID_calc(&pid_v1[i-1],motor_can1[i-1].speed_rpm , set_speed);
		return pid_v1[i-1].out;
}

float PID_position_realize_1(float set_pos,int i)
{
		PID_calc(&pid_pos1[i-1],motor_can1[i-1].total_angle , set_pos);
		return pid_pos1[i-1].out;

}

float PID_force_realize_1(float set_force,int i){
	PID_calc(&pid_force[i-1],motor_can1[i-1].given_current,set_force);
	return pid_force[i-1].out;
}



         /***********can2电机PID***************/
float PID_velocity_realize_2(float set_speed,int i)
{
		PID_calc(&pid_v2[i-1],motor_can2[i-1].speed_rpm , set_speed);
		return pid_v2[i-1].out;
}

float PID_position_realize_2(float set_pos,int i)
{
		PID_calc(&pid_pos2[i-1],motor_can2[i-1].total_angle , set_pos);
		return pid_pos2[i-1].out;

}

float PID_force_realize_2(float set_force,int i){
	PID_calc(&pid_force[i-1],motor_can2[i-1].given_current,set_force);
	return pid_force[i-1].out;
}



         /***********can1电机双环PID***************/
float pid_call_1(float position,int i)
{
		return PID_velocity_realize_1(PID_position_realize_1(position,i),i);
}
         /***********can2电机双环PID***************/
float pid_call_2(float position,int i)
{
		return PID_velocity_realize_2(PID_position_realize_2(position,i),i);
}



extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

motor_measure_t motor_can1[8];
motor_measure_t motor_can2[8];

CAN_TxHeaderTypeDef can_tx_message;
uint8_t	can_send_data[8];

void get_motor_measure(motor_measure_t *ptr,uint8_t data[])                                                     
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
		
		
		
void get_moto_offset(motor_measure_t *ptr, uint8_t data[])
{
	ptr->angle = data[0]<<8 |data[1] ;
	ptr->offset_angle = ptr->angle;
}



#define ABS(x)	( (x>0) ? (x) : (-x) )


void get_total_angle(motor_measure_t *p)
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
	


		uint8_t rx_data_1[8];
		uint8_t rx_data_2[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		CAN_RxHeaderTypeDef rx_header;

	
	if(hcan->Instance==CAN1)
	{
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data_1);

		switch (rx_header.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:

			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_3508_M1_ID;
				motor_can1[i].msg_cnt++ <= 50	?	get_moto_offset(&motor_can1[i], rx_data_1) : get_motor_measure(&motor_can1[i], rx_data_1);
				get_motor_measure(&motor_can1[i], rx_data_1);
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

//			case CAN_3508_ALL_ID:
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_3508_M1_ID;
				motor_can2[i].msg_cnt++ <= 50	?	get_moto_offset(&motor_can2[i], rx_data_2) : get_motor_measure(&motor_can2[i], rx_data_2);
				get_motor_measure(&motor_can2[i], rx_data_2);
				break;
			}
			default:
			{
			break;
			}	
		}
 }
}

 
 
 
 void CAN1_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
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




 void CAN1_CMD_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
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
 HAL_CAN_AddTxMessage(&hcan1, &can_tx_message, can_send_data, &send_mail_box);
}



 void CAN2_CMD_1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
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


 void CAN2_CMD_2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
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



void filter1_init(void)
{
	CAN_FilterTypeDef filter1;
	filter1.FilterBank=0;//滤波器编
	filter1.FilterMode=CAN_FILTERMODE_IDMASK;//掩码模式
	filter1.FilterScale=CAN_FILTERSCALE_32BIT;
	filter1.FilterIdHigh=0x0000;
	filter1.FilterIdLow=0x0000;
	filter1.FilterMaskIdHigh=0x0000;
	filter1.FilterMaskIdLow=0x0000;
	filter1.FilterFIFOAssignment=CAN_FILTER_FIFO0;//FIFO0
	filter1.FilterActivation=ENABLE;
	
	if(HAL_CAN_ConfigFilter(&hcan1,&filter1)!=HAL_OK)
	{
		Error_Handler();
	}
}

void filter2_init(void)
{
	  CAN_FilterTypeDef CAN_Filter_st;
    CAN_Filter_st.FilterActivation = ENABLE;
    CAN_Filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_Filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_Filter_st.FilterIdHigh = 0x0000;
    CAN_Filter_st.FilterIdLow = 0x0000;
    CAN_Filter_st.FilterMaskIdHigh = 0x0000;
    CAN_Filter_st.FilterMaskIdLow = 0x0000;
    CAN_Filter_st.FilterBank = 14;
	  CAN_Filter_st.SlaveStartFilterBank=14;
    CAN_Filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	  HAL_CAN_ConfigFilter(&hcan2, &CAN_Filter_st);        //滤波器初始化
}


