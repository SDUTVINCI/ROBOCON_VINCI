#ifndef _chassis_
#define _chassis_

#include "include.h"

//void chassis_task(void const *argument);

// 任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 200

// 底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2

typedef struct
{

        float theta[3]; // 轮方向																										rad
        float V[3];     // 轮速度																										mm/s

} wheel_t;
typedef struct
{

        float Vx, Vy; // 轮方向																										rad
        float Vomega; // 轮速度																										mm/s

} chassis_t;



int16_t tansform_6020(float theta);
wheel_t Formula(chassis_t chassis);
chassis_t Formula_World2Robo(chassis_t world,fp32 theta) ;
#endif
