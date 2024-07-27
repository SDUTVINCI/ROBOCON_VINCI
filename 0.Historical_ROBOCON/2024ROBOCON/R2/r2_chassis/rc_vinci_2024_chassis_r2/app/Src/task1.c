#include "include.h"
#include "data_pack.h"
extern fp32 yaw;
extern uint8_t cmd_myfromActuator;
extern uint8_t rx_datafromActuator[10];
extern uint8_t rx_datafromMapan[10];
extern uint8_t cmd_myfromMapan;
extern fp32 x_mapan;
extern fp32 y_mapan;
extern osThreadId scheduler_taskHandle;
extern osThreadId jiaozhun_taskHandle;
extern osThreadId mapanmoveHandle;
extern osThreadId daokuangtaskHandle;
extern osThreadId l1smovepointHandle;
extern uint8_t move_mode;
extern uint8_t if_jiaozhun_finish;


extern uint8_t move_mode; // 是否使用遥控器标志位  默认为1使用遥控器
#include "imu605.h"
// #include "L1S.h"
float world_vx, world_vy, world_vomega; // 控制机器人x,y,omega速度

//extern RC_ctrl_t rc_ctrl;
void robot_move(float vx, float vy, float vomega);

extern pid_type_def pid_v_chassis_3508[3], pid_v_chassis_6020[3], pid_p_chassis_6020[3];
extern imu605_t imu605;
fp32 world_omega = 0;
void move_chassis_task(void const *argument)
{
	//	static uint8_t point_step[2] = {0, 1}; //[0]  上次step    [1]now_step

	// 空闲一段时间
	//	vTaskDelay(CHASSIS_TASK_INIT_TIME); // 上电初始化  马盘初始化等待时间
	// HEX_FastConti_Meas_Cmd();//l1s快速连续测量
	// uint8_t flage1 = 40;
	for (;;)
	{
		if (move_mode == 1) // 不使用遥控器
		{
			robot_move(world_vx, world_vy, world_vomega);
		}
//		else if (move_mode == 0) // 使用遥控器
//		{
//			robot_move_remote();
//		}
		else if (move_mode == 2)
		{
			world_omega = -90;
			robot_move(world_vx, world_vy, -PID_position_chassis_omega1(world_omega));
		}
		osDelay(7);
	}
}

//#define Translation_Speed_F 9 // 平移速度系数
//#define Rotation_Speed_F 1
// 遥控模式 0，0，0  时   舵电机无力
// void move_mode1(void)
//{
//	chassis_t chassis_r1;
//	wheel_t wheel_r1;
//	chassis_r1.Vx = -rc_ctrl.rc.ch[0] * 2 * Translation_Speed_F;
//	chassis_r1.Vy = -rc_ctrl.rc.ch[1] * 2 * Translation_Speed_F;
//	chassis_r1.Vomega = rc_ctrl.rc.ch[2] / 60 * Rotation_Speed_F;
//	if (chassis_r1.Vy == 0 && chassis_r1.Vx == 0 && chassis_r1.Vomega == 0)
//	{
//		gm6020_cmd(PID_velocity_realize_6020(0, 1), PID_velocity_realize_6020(0, 2), PID_velocity_realize_6020(0, 3), 0);
//		gm3508_cmd(PID_velocity_realize_chassis_3508(0, 1), PID_velocity_realize_chassis_3508(0, 2), PID_velocity_realize_chassis_3508(0, 3), 0);
//	}
//	else
//	{
//		wheel_r1 = Formula(chassis_r1);

//		gm6020_cmd(PID_call_6020(tansform_6020(wheel_r1.theta[0]), 1), PID_call_6020(tansform_6020(wheel_r1.theta[1]), 2), PID_call_6020(tansform_6020(wheel_r1.theta[2]), 3), 0);
//		gm3508_cmd(PID_velocity_realize_chassis_3508(wheel_r1.V[0] * cos(pid_p_chassis_6020[0].error[0] / 1305), 1), PID_velocity_realize_chassis_3508(wheel_r1.V[1] * cos(pid_p_chassis_6020[1].error[0] / 1305), 2), PID_velocity_realize_chassis_3508(wheel_r1.V[2] * cos(pid_p_chassis_6020[2].error[0] / 1305), 3), 0);
//	}
//}
// 遥控模式   0，0，0  时   舵电机自动回到固定位置
// void move_mode2(void)
//{
//	chassis_t chassis_r1_world;
//	chassis_r1_world.Vx = -rc_ctrl.rc.ch[0] * 5;
//	chassis_r1_world.Vy = -rc_ctrl.rc.ch[1] * 5;
//	chassis_r1_world.Vomega = rc_ctrl.rc.ch[2] / 60;
//	chassis_t chassis_r1_robot;
//	wheel_t wheel_r1;
//	chassis_r1_robot = Formula_World2Robo(chassis_r1_world, yaw / 57.32);

//	wheel_r1 = Formula(chassis_r1_robot);
//	gm6020_cmd(PID_call_6020(tansform_6020(wheel_r1.theta[0]), 1), PID_call_6020(tansform_6020(wheel_r1.theta[1]), 2), PID_call_6020(tansform_6020(wheel_r1.theta[2]), 3), 0);
//	gm3508_cmd(PID_velocity_realize_chassis_3508(wheel_r1.V[0] * cos(pid_p_chassis_6020[0].error[0] / 1305), 1), PID_velocity_realize_chassis_3508(wheel_r1.V[1] * cos(pid_p_chassis_6020[1].error[0] / 1305), 2), PID_velocity_realize_chassis_3508(wheel_r1.V[2] * cos(pid_p_chassis_6020[2].error[0] / 1305), 3), 0);
//}

// 遥控模式   世界坐标系转换成机器人坐标系
// void move_mode3(void)
//{
//	chassis_t chassis_r1_world;
//	chassis_t chassis_r1_robot;
//	wheel_t wheel_r1;

//	chassis_r1_world.Vx = -rc_ctrl.rc.ch[0];
//	chassis_r1_world.Vy = -rc_ctrl.rc.ch[1];
//	chassis_r1_world.Vomega = rc_ctrl.rc.ch[2] / 60;

//	chassis_r1_robot = Formula_World2Robo(chassis_r1_world, IMU_YAW / 57.32);

//	wheel_r1 = Formula(chassis_r1_robot);
//	gm6020_cmd(PID_call_6020(tansform_6020(wheel_r1.theta[0]), 1), PID_call_6020(tansform_6020(wheel_r1.theta[1]), 2), PID_call_6020(tansform_6020(wheel_r1.theta[2]), 3), 0);
//	gm3508_cmd(PID_velocity_realize_chassis_3508(wheel_r1.V[0] * cos(pid_p_chassis_6020[0].error[0] / 1305), 1), PID_velocity_realize_chassis_3508(wheel_r1.V[1] * cos(pid_p_chassis_6020[1].error[0] / 1305), 2), PID_velocity_realize_chassis_3508(wheel_r1.V[2] * cos(pid_p_chassis_6020[2].error[0] / 1305), 3), 0);
//}

// 遥控模式 0，0，0  时   舵电机无力
//void robot_move_remote(void)
//{
//	chassis_t chassis_r1_world;
//	chassis_t chassis_r1_robot;

//	wheel_t wheel_r1;
//	chassis_r1_world.Vx = -rc_ctrl.rc.ch[0] * 2 * 2;
//	chassis_r1_world.Vy = -rc_ctrl.rc.ch[1] * 2 * 2;
//	chassis_r1_world.Vomega = rc_ctrl.rc.ch[2] / 90;

//	chassis_r1_robot = Formula_World2Robo(chassis_r1_world, yaw / 57.32);

//	if (chassis_r1_robot.Vy == 0 && chassis_r1_robot.Vx == 0 && chassis_r1_robot.Vomega == 0)
//	{
//		gm6020_cmd(PID_velocity_realize_6020(0, 1), PID_velocity_realize_6020(0, 2), PID_velocity_realize_6020(0, 3), 0);
//		gm3508_cmd(PID_velocity_realize_chassis_3508(0, 1), PID_velocity_realize_chassis_3508(0, 2), PID_velocity_realize_chassis_3508(0, 3), 0);
//	}
//	else
//	{
//		wheel_r1 = Formula(chassis_r1_robot);
//		gm6020_cmd(PID_call_6020(tansform_6020(wheel_r1.theta[0]), 1), PID_call_6020(tansform_6020(wheel_r1.theta[1]), 2), PID_call_6020(tansform_6020(wheel_r1.theta[2]), 3), 0);
//		gm3508_cmd(PID_velocity_realize_chassis_3508(wheel_r1.V[0] * cos(pid_p_chassis_6020[0].error[0] / 1305), 1), PID_velocity_realize_chassis_3508(wheel_r1.V[1] * cos(pid_p_chassis_6020[1].error[0] / 1305), 2), PID_velocity_realize_chassis_3508(wheel_r1.V[2] * cos(pid_p_chassis_6020[2].error[0] / 1305), 3), 0);
//	}
//}
//void move_mode5(void)
//{
//	chassis_t chassis_r1_world;
//	chassis_t chassis_r1_robot;

//	wheel_t wheel_r1;
//	chassis_r1_world.Vx = -rc_ctrl.rc.ch[0] * 2 * 9;
//	chassis_r1_world.Vy = -rc_ctrl.rc.ch[1] * 2 * 9;
//	chassis_r1_world.Vomega = rc_ctrl.rc.ch[2] / 60;

//	chassis_r1_robot = Formula_World2Robo(chassis_r1_world, yaw / 57.32);

//	if (chassis_r1_robot.Vy == 0 && chassis_r1_robot.Vx == 0 && chassis_r1_robot.Vomega == 0)
//	{
//		gm6020_cmd(PID_velocity_realize_6020(0, 1), PID_velocity_realize_6020(0, 2), PID_velocity_realize_6020(0, 3), 0);
//		gm3508_cmd(PID_velocity_realize_chassis_3508(0, 1), PID_velocity_realize_chassis_3508(0, 2), PID_velocity_realize_chassis_3508(0, 3), 0);
//	}
//	else
//	{
//		wheel_r1 = Formula(chassis_r1_robot);
//		gm6020_cmd(PID_call_6020(tansform_6020(wheel_r1.theta[0]), 1), PID_call_6020(tansform_6020(wheel_r1.theta[1]), 2), PID_call_6020(tansform_6020(wheel_r1.theta[2]), 3), 0);
//		gm3508_cmd(PID_velocity_realize_chassis_3508(wheel_r1.V[0] * cos(pid_p_chassis_6020[0].error[0] / 1305), 1), PID_velocity_realize_chassis_3508(wheel_r1.V[1] * cos(pid_p_chassis_6020[1].error[0] / 1305), 2), PID_velocity_realize_chassis_3508(wheel_r1.V[2] * cos(pid_p_chassis_6020[2].error[0] / 1305), 3), 0);
//	}
//}
void robot_move(float vx, float vy, float vomega)
{
	chassis_t chassis_r1_world;
	chassis_t chassis_r1_robot;

	wheel_t wheel_r1;
	chassis_r1_world.Vx = vx;
	chassis_r1_world.Vy = vy;
	chassis_r1_world.Vomega = vomega;

	chassis_r1_robot = Formula_World2Robo(chassis_r1_world, yaw / 57.32);

	if (chassis_r1_robot.Vy == 0 && chassis_r1_robot.Vx == 0 && chassis_r1_robot.Vomega == 0)
	{
		gm6020_cmd(PID_velocity_realize_6020(0, 1), PID_velocity_realize_6020(0, 2), PID_velocity_realize_6020(0, 3), 0);
		gm3508_cmd(PID_velocity_realize_chassis_3508(0, 1), PID_velocity_realize_chassis_3508(0, 2), PID_velocity_realize_chassis_3508(0, 3), 0);
	}
	else
	{
		wheel_r1 = Formula(chassis_r1_robot);
		gm6020_cmd(PID_call_6020(tansform_6020(wheel_r1.theta[0]), 1), PID_call_6020(tansform_6020(wheel_r1.theta[1]), 2), PID_call_6020(tansform_6020(wheel_r1.theta[2]), 3), 0);
		gm3508_cmd(PID_velocity_realize_chassis_3508(wheel_r1.V[0] * cos(pid_p_chassis_6020[0].error[0] / 1305), 1), PID_velocity_realize_chassis_3508(wheel_r1.V[1] * cos(pid_p_chassis_6020[1].error[0] / 1305), 2), PID_velocity_realize_chassis_3508(wheel_r1.V[2] * cos(pid_p_chassis_6020[2].error[0] / 1305), 3), 0);
	}
}
// 到定点以一定角度   未到达返回0，到达位置返回1
// uint8_t move_mode4(int world_x, int world_y, float omega) // 到定点
//{

//	if ((omega - IMU_YAW > 4 || omega - IMU_YAW < -4) || (world_x - x > 3 || world_x - x < -3) || (world_y - y > 3 || world_y - y < -3))
//	{

//		chassis_t chassis_r1_world;		        // 世界坐标系
//		chassis_t chassis_r1_robot;		        // 机器人坐标系
//		wheel_t wheel_r1;		        // 轮系
//		chassis_r1_world.Vx = -PID_position_chassis_x(world_x); // position
//		chassis_r1_world.Vy = -PID_position_chassis_y(world_y); // position
//		chassis_r1_world.Vomega = -PID_position_chassis_omega1(omega);
//		chassis_r1_robot = Formula_World2Robo(chassis_r1_world, IMU_YAW / 57.32);
//		wheel_r1 = Formula(chassis_r1_robot);
//		gm6020_cmd(PID_call_6020(tansform_6020(wheel_r1.theta[0]), 1), PID_call_6020(tansform_6020(wheel_r1.theta[1]), 2), PID_call_6020(tansform_6020(wheel_r1.theta[2]), 3), 0);
//		gm3508_cmd(PID_velocity_realize_chassis_3508(wheel_r1.V[0] * cos(pid_p_chassis_6020[0].error[0] / 1305), 1), PID_velocity_realize_chassis_3508(wheel_r1.V[1] * cos(pid_p_chassis_6020[1].error[0] / 1305), 2), PID_velocity_realize_chassis_3508(wheel_r1.V[2] * cos(pid_p_chassis_6020[2].error[0] / 1305), 3), 0);
//		return 0;
//	}

//	else
//	{
//		gm6020_cmd(PID_velocity_realize_6020(0, 1), PID_velocity_realize_6020(0, 2), PID_velocity_realize_6020(0, 3), 0);
//		gm3508_cmd(PID_velocity_realize_chassis_3508(0, 1), PID_velocity_realize_chassis_3508(0, 2), PID_velocity_realize_chassis_3508(0, 3), 0);

//		return 1;
//	}
//}

// 到定点以一定角度   未到达返回0，到达位置返回1
// uint8_t move_mode4(int world_x, int world_y, float omega) // 到定点
//{

//	if ((omega - IMU_YAW > 4 || omega - IMU_YAW < -4) || (world_x - ops9.Position.pos_x > 3 || world_x - ops9.Position.pos_x < -3) || (world_y - ops9.Position.pos_y > 3 || world_y - ops9.Position.pos_y < -3))
//	{

//		chassis_t chassis_r1_world;		        // 世界坐标系
//		chassis_t chassis_r1_robot;		        // 机器人坐标系
//		wheel_t wheel_r1;		        // 轮系
//		chassis_r1_world.Vx = -PID_position_chassis_x(world_x); // position
//		chassis_r1_world.Vy = -PID_position_chassis_y(world_y); // position
//		chassis_r1_world.Vomega = -PID_position_chassis_omega(omega);
//		chassis_r1_robot = Formula_World2Robo(chassis_r1_world, IMU_YAW / 57.32);
//		wheel_r1 = Formula(chassis_r1_robot);
//		gm6020_cmd(PID_call_6020(tansform_6020(wheel_r1.theta[0]), 1), PID_call_6020(tansform_6020(wheel_r1.theta[1]), 2), PID_call_6020(tansform_6020(wheel_r1.theta[2]), 3), 0);
//		gm3508_cmd(PID_velocity_realize_chassis_3508(wheel_r1.V[0] * cos(pid_p_chassis_6020[0].error[0] / 1305), 1), PID_velocity_realize_chassis_3508(wheel_r1.V[1] * cos(pid_p_chassis_6020[1].error[0] / 1305), 2), PID_velocity_realize_chassis_3508(wheel_r1.V[2] * cos(pid_p_chassis_6020[2].error[0] / 1305), 3), 0);
//		return 0;
//	}

//	else
//	{
//		gm6020_cmd(PID_velocity_realize_6020(0, 1), PID_velocity_realize_6020(0, 2), PID_velocity_realize_6020(0, 3), 0);
//		gm3508_cmd(PID_velocity_realize_chassis_3508(0, 1), PID_velocity_realize_chassis_3508(0, 2), PID_velocity_realize_chassis_3508(0, 3), 0);

//		return 1;
//	}
//
//}
void Mapan_Init(void)
{
	uint8_t tx_buf_mapan = 0x76;
	SendPakageWaitACK(Mapan, 0x69, &tx_buf_mapan, 1, 0x01);
}

// points_number点的总数量
// points[][3]存放点的数组
// move_points_stations move_points(uint8_t points_number, int16_t points[][3])
//{
//	move_points_stations state;
//	static uint8_t step = 0;
//	uint8_t flage = 0;
//	if (step == points_number) // 到达最终点  返回0
//	{
//		move_mode4(points[points_number - 1][0], points[points_number - 1][1], points[points_number - 1][2]);
//		return 0;
//	}
//	else // 未到达最终点  返回正在追的点的排序
//	{
//		flage = move_mode4(points[step][0], points[step][1], points[step][2]);
//		if (flage)
//		{
//			step += 1;
//		}
//		if (step == points_number)
//		{
//			return 0;
//		}
//		return step + 1;
//	}
//}

void Task_LED(void const *argument)
{
	// 空闲一段时间
	vTaskDelay(CHASSIS_TASK_INIT_TIME); // 上电初始化  马盘初始化等待时间

	for (;;)
	{

		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_11);

		osDelay(200);
	}
}

//int ddlt = 0;

// uint8_t Gyro_action(fp64 omega)
//{
//	for (;;)
//	{
//		if (fabs(yaw - omega) < 2)
//		{
//			world_vx = 0;
//			world_vy = 0;
//			world_vomega = 0;
//			return 1;
//
//		}
//
//			///
//		world_vomega = -PID_position_chassis_omega1(omega);
//		world_vx = 0;
//		world_vy = 0;
//		return 0;
//	}
// }

uint8_t Gyro_action(fp64 omega)
{
	for (;;)
	{

		if (fabs(yaw - omega) < 2.0)
		{
			move_mode = 1;
			world_vx = 0.0;
			world_vy = 0.0;
			world_vomega = 0.0;
			return 1;
		}
		move_mode = 2;
		world_vx = 0;
		world_vy = 0;
		return 0;
	}
}
// extern double angle;
extern unsigned int gdistance_X2 ;
extern int att;
extern unsigned int gdistance_X ;
uint8_t jiaozhun_action(void)
{
	move_mode = 1;
	osDelay(20);
	static uint8_t delay = 0;
	if (abs((int)gdistance_X + 75 - (int)gdistance_X2) <4)
	{
		delay++;
		if (delay > 2)
		{
			world_vomega = 0;
			world_vx = 0;
			world_vy = 0;
			return 1; /* code */
		}
	}
	else
	{
		delay = 0;
	}

	world_vomega = PID_position_L1S_jiaozhun();
	world_vx = 0;
	world_vy = 0;
	return 0;
}
uint8_t if_mapan_move_point_task_finish = 0;
uint8_t Mapan_move_postion(fp32 x_of_3port, fp32 y_of_3port, fp32 omega) // 世界坐标系
{
	move_mode = 2;

	if (( (fabs(x_mapan - x_of_3port)) < 20.0) && ((fabs(y_mapan - y_of_3port)) < 20.0))
	{
world_vx = 0;
	world_vy = 0;
		// world_vomega = 0;
		if_mapan_move_point_task_finish = 1;
		return 1;
	}
	else{
	if_mapan_move_point_task_finish = 0;
	world_vx = -PID_position_Mapan_y(y_of_3port);
	world_vy = PID_position_Mapan_x(x_of_3port);
	return 0;
	}
	
	// world_vomega = -PID_position_chassis_omega1(omega);
	
}
fp32 x_point_mapan[7] = {0.0, 0.0, 400, 200, 0}; //[0]  初始零点
fp32 y_point_mapan[7] = {0.0, -2600.0, -3000, -2800, 0};
uint8_t step_mapan = 0; // 0: 零点  1：能看到所有框的点 2：一号框位置  3：二号框位置
// 标志位（使用前需清零） if_mapan_move_point_task_finish
void mapan_move_point_task(void const *argument)
{

	for (;;)
	{
		move_mode = 2;

		if (Mapan_move_postion(x_point_mapan[step_mapan], y_point_mapan[step_mapan], -90))
		{
			vTaskSuspend(NULL);
		}

		osDelay(6);
	}
}
extern unsigned int gdistance_X;
extern unsigned int gdistance_Y;
uint8_t if_l1s_move_point_task_finish = 0;		     // 任务完成标制位
uint8_t L1S_move_postion(int x_of_3port, int y_of_3port, fp32 omega) // imu校准方向控制在三区坐标系下的位置
{
	move_mode = 2;
	static int delay_time = 0;
//	if (fabs(yaw + 90) > 5)
//	{
//		world_vx = 0;
//		world_vy = 0;
//	//	// world_vomega = -PID_position_chassis_omega1(omega);
//	}
//	else
//	{
		world_vx = PID_position_L1S_y(y_of_3port);
		world_vy = -PID_position_L1S_x(x_of_3port);
//		// world_vomega = -PID_position_chassis_omega1(omega);
//	}

	//if ((fabs(yaw - omega) < 0.5) && ((abs((int)(gdistance_X - x_of_3port))) < 10) && ((abs((int)(gdistance_Y - y_of_3port))) < 10))
	if ( (abs((int)(gdistance_X - x_of_3port)) < 10) && (abs((int)(gdistance_Y - y_of_3port)) < 10))
	{
		
			if_l1s_move_point_task_finish = 1;
			delay_time = 0;
			//      world_vomega = 0;
			//      world_vy = 0;
			//      world_vx = 0;
			// vTaskSuspend(jiaozhun_taskHandle);
			return 1;
		
	}

	if_l1s_move_point_task_finish = 0;

	return 0;
}
uint8_t target_kuang = 0; // 0：一号框位置    5：零点位置
int x_port_l1s[6] = {3248, 2498, 1748, 998, 238, 1750};
int y_port_l1s[6] = {350, 350, 350, 350, 350, 1200};
void l1s_move_point_task(void const *argument)
{

	for (;;)
	{

		if (L1S_move_postion(x_port_l1s[target_kuang], y_port_l1s[target_kuang], -90))
		{

			vTaskSuspend(NULL);
		}

		osDelay(6);
	}
}
uint8_t station=0;

extern  osThreadId firstjiaozhunHandle;

void daokuang_task(void const *argument)
{

	for (;;)
	{
		uint8_t txbuf;
		move_mode=2;
		osDelay(20);
		// step_mapan
		// target_kuang
		if_mapan_move_point_task_finish = 0;
		if_l1s_move_point_task_finish = 0;
		if_jiaozhun_finish = 0;HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1);

		
		do{
	
		osDelay(10);
		station=Mapan_move_postion(x_point_mapan[step_mapan], y_point_mapan[step_mapan], -90);
		}
		while(!station);
		world_vx=0;
		world_vy=0;
		
		
		
		
		vTaskResume(jiaozhun_taskHandle);
		while (!if_jiaozhun_finish)
		{
			osDelay(10);
		}
		

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10,0);
		
		
		
		while (!(L1S_move_postion(x_port_l1s[target_kuang], y_port_l1s[target_kuang], -90)))
		{
			osDelay(10);
			
		}
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,1);



		txbuf = 0x00;
		SendPakageWaitACK(Actuator, 0x50, &txbuf, 1, 0xff);

		

		vTaskSuspend(NULL);
	}
}
extern fp32 yaw_offset;
extern fp32 x_offset;
extern fp32 y_offset;
uint8_t if_jiaozhun_finish = 0;
void jiaozhuntask(void const *argument)
{

	for (;;)
	{
		if_jiaozhun_finish = 0;
		while (!Gyro_action(-90.0))
		{
			osDelay(6);
		}
		while (!jiaozhun_action())
		{

			osDelay(6);
		}

		osDelay(500);
		yaw_offset = yaw + 90;
		uint8_t tx_buf_mapan = 0x76;

		x_offset = (fp32)2000.0 - ((fp32)gdistance_X + (fp32)250.0);
		y_offset = (fp32)1550.0 - ((fp32)gdistance_Y + (fp32)180.0);
		// uint8_t txbuf = 0x00;
		// Send_Cmd_Data2Actuator(0x48, &txbuf, 1);
		if_jiaozhun_finish = 1;
		vTaskSuspend(NULL);
	}
}
//  在当前位置  使用激光测距校准后  使用激光测距移动到某点
//  target_kuang  控制校准后到的坐标位置点
//  任务完成返送 0x48 0x00;
void first_jiaozhun_task(void const *argument)
{

	for (;;)
	{
if_jiaozhun_finish = 0;if_l1s_move_point_task_finish = 0;
		move_mode = 1;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 1);
		vTaskResume(jiaozhun_taskHandle);
		while (!if_jiaozhun_finish)
		{
			osDelay(10);
		}
		//if_jiaozhun_finish = 0;
		target_kuang = 5;
		osDelay(20);
		move_mode = 2;
		osDelay(20);
		
		vTaskResume(l1smovepointHandle);
		
		while (if_l1s_move_point_task_finish == 0)
		{
			osDelay(10);
		}
		//if_l1s_move_point_task_finish = 0;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10,1);
		world_vx = 0;
		world_vy = 0;
		Mapan_Init();
		osDelay(300);
		y_offset = 0;
		x_offset = 0;
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10,0);
		uint8_t buf = 0x00;
		SendPakageWaitACK(Actuator, 0x48, &buf, 1, 0xff); // 位置校准完毕
		vTaskSuspend(NULL);
	}
}

void grya_Task(void const *argument)
{

	for (;;)
	{
		while (!Gyro_action(-90.0))
		{
			osDelay(6);
		}
		vTaskSuspend(NULL);
	}
}