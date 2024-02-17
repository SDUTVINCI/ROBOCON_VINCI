#include "chassis_task.h"
#include "global_position.h"

extern RC_ctrl_t rc_ctrl;

extern Chassis_Speed_t absolute_chassis_speed;
extern osSemaphoreId PC_BinarySemHandle;		
extern bool_t GP_SWITCH_STATUS;
void chassis_task(void const * argument)
{
	// wait a time
	//空闲一段时间
	vTaskDelay(CHASSIS_TASK_INIT_TIME);

	//PID parameter initialization
	//PID参数初始化
	PID_devices_Init();
	while(1)
	{
//		//obtaining attitude data semaphore
//		//获取姿态数据信号量
//		xSemaphoreTake(IMU_Chassis_BinarySemHandle,portMAX_DELAY);
		
		//set chassis motion mode
		//设定底盘运动模式
		Remote_Control_Chassis_Set_Mode();

		//chassis data update
		//底盘数据更新
		Remote_Control_Chassis_Mode(&absolute_chassis_speed);
		
		//chassis pose analysis
		//底盘姿态分析
		Robot_Pose_Analysis();
		if(GP_SWITCH_STATUS == 0)
		{
		//calculate the speed of the chassis
		//底盘速度解算
		Chassis_Sports_Calc(absolute_chassis_speed);

		//chassis control pid calculate
		//底盘控制PID计算与数据发送
		Chassis_Loop_Out();
		
//		xSemaphoreGive(PC_BinarySemHandle);

	  }
		else
		{
			OPS9_GP_Stage1();
		}

		//chassis task control time
		//底盘任务控制间隔
		osDelay(CHASSIS_CONTROL_TIME_MS);
 }
}
