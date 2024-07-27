#ifndef _chassis_
#define _chassis_

#include "include.h"

//void chassis_task(void const *argument);

// ����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 200

// ����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2

typedef struct
{

        float theta[3]; // �ַ���																										rad
        float V[3];     // ���ٶ�																										mm/s

} wheel_t;
typedef struct
{

        float Vx, Vy; // �ַ���																										rad
        float Vomega; // ���ٶ�																										mm/s

} chassis_t;



int16_t tansform_6020(float theta);
wheel_t Formula(chassis_t chassis);
chassis_t Formula_World2Robo(chassis_t world,fp32 theta) ;
#endif
