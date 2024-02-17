#ifndef __CHASSIS_TASK_H_
#define __CHASSIS_TASK_H_
#include "include.h"

void chassis_task(void const * argument);

//in the beginning of task ,wait a time
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

//chassis task control time  2ms
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2


#endif


