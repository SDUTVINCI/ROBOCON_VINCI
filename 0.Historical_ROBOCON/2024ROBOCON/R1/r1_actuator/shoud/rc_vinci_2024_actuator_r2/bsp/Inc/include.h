#ifndef __INCLUDE_H_
#define __INCLUDE_H_

#include "main.h"
#include "cmsis_os.h"
#include "can.h"
//#include "dma.h"
//#include "tim.h"
//#include "usart.h"
#include "gpio.h"
#include "data_pack.h"
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "bsp_can.h"
#include "pid.h"
#include "pid_user.h"
//#include "imu_ops9.h"
//#include "bsp_rc.h"
//#include "remote_control.h"
//#include "chassis_api.h"
//#include "chassis_task.h"
#include <math.h>
#include <string.h>
#include "chassis.h"



#define IMU_YAW 	imu605.Euler.yaw
#define IMU_ROLL	imu605.Euler.roll
#define IMU_PITCH	imu605.Euler.pitch

#define WORLD_POSTION_X	0 
#define WORLD_POSTION_Y	0
#define WORLD_POSTION_Z	0


#endif


