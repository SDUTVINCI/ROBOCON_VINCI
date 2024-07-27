#include "include.h"
extern unsigned int gdistance_Y2;
extern unsigned int gdistance_X2;
extern unsigned int gdistance_Y1;
extern unsigned int gdistance_X1;
#include <stdlib.h>
#include <math.h>
// int16_t points[30][3] = {
//     {0, 0, 0},
//     {300, 300, 0},
//     {750, 280},
//     {1000, 250, 0},
//     {1250, 250, 0},
//     {1500, 250, 0},
//     {1750, 250, 0},
//     {2000, 250, 0},
//     {2250, 250, 0},
//     {1125, 2850, 0},
//     {1625, 2850, 0},
//     {2125, 2850, 0},
//     {2625, 2850, 0},
//     {3125, 2850, 0},
//     {3625, 2850, 0},

//    {4120, 2850, 0},
//    {4120, 1200},

//    {2500, 250, 0},
//    {2750, 250, 0},
//    {3000, 250, 0},
//    {3250, 250, 0},
//    {3500, 250, 0},
//    {3750, 250, 0},
//    {3700, 1700, 60},
//    {3200, 1700, 60},
//    {2700, 1700, 60},
//    {2200, 1700, 60},
//    {1700, 1700, 60},
//    {1200, 1700, 60},
//    {10, 50, 0}};

// extern ops9_t ops9;

// typedef struct
//{
//	int8_t state;	   // 0：未到达new_point    1:到达new_point
//	int8_t last_point; // 上一个point标号
//	int8_t new_point;  // 下一个或此时所在的point标号

//} move_points_stations;

// uint8_t move_mode4(int world_x, int world_y, float omega);
// move_points_stationsss move_points(uint8_t points_number, int16_t points[][3]);

extern int angol_remote[4];
#include "imu605.h"
#include "L1S.h"
extern uint8_t nrf_key[20];
void move_mode_v(int vx, int vy, int vomega);
extern fp32 yaw;
extern pid_type_def pid_v_chassis_3508[3], pid_v_chassis_6020[3], pid_p_chassis_6020[3];
extern imu605_t imu605;
uint8_t tx_buffer[1];
uint8_t move_mode4();
uint8_t move_mode_flage = 0;
int Vx, Vy, omega;
void move_mode_nrf(void);
void StartTask02(void const *argument)
{

	for (;;)
	{

		Vx = angol_remote[0] * 6;
		Vy = -angol_remote[1] * 6;
		omega = angol_remote[2] / 4;
		// omega=1.0;
		move_mode_v(Vx, Vy, PID_position_omega_speed(omega));

		osDelay(6);
	}
}
int ttttttt;
#define Translation_Speed_F 9 // 平移速度系数
#define Rotation_Speed_F 1
// 遥控模式 0，0，0  时   舵电机无力
// void move_mode1(void)
//{
//	chassis_t chassis_r1;
//	wheel_t wheel_r1;
//	chassis_r1.Vx = -rc_ctrl.rc.ch[0] * 2*Translation_Speed_F;
//	chassis_r1.Vy = -rc_ctrl.rc.ch[1] * 2*Translation_Speed_F;
//	chassis_r1.Vomega = rc_ctrl.rc.ch[2] / 60*Rotation_Speed_F;
//	if (chassis_r1.Vy == 0 && chassis_r1.Vx == 0 && chassis_r1.Vomega == 0)
//	{
//		gm6020_cmd(PID_velocity_realize_6020(0, 1), PID_velocity_realize_6020(0, 2), PID_velocity_realize_6020(0, 3), 0);
//		gm3508_cmd(PID_velocity_realize_chassis_3508(0, 1), PID_velocity_realize_chassis_3508(0, 2), PID_velocity_realize_chassis_3508(0, 3), 0);
//	}
//	else
//	{
//		wheel_r1= Formula(chassis_r1);
//
//		gm6020_cmd(PID_call_6020(tansform_6020(wheel_r1.theta[0]), 1), PID_call_6020(tansform_6020(wheel_r1.theta[1]), 2), PID_call_6020(tansform_6020(wheel_r1.theta[2]), 3), 0);
//		gm3508_cmd(PID_velocity_realize_chassis_3508(wheel_r1.V[0] * cos(pid_p_chassis_6020[0].error[0] / 1305), 1), PID_velocity_realize_chassis_3508(wheel_r1.V[1] * cos(pid_p_chassis_6020[1].error[0] / 1305), 2), PID_velocity_realize_chassis_3508(wheel_r1.V[2] * cos(pid_p_chassis_6020[2].error[0] / 1305), 3), 0);
//	}
// }
// 遥控模式   0，0，0  时   舵电机自动回到固定位置
// void move_mode2(void)
//{
//	chassis_t chassis_r1_world;
//	chassis_r1_world.Vx = -rc_ctrl.rc.ch[0] * 5;
//	chassis_r1_world.Vy = -rc_ctrl.rc.ch[1] * 5;
//	chassis_r1_world.Vomega = rc_ctrl.rc.ch[2] / 60;
//	chassis_t chassis_r1_robot;
//	wheel_t wheel_r1;
//	chassis_r1_robot = Formula_World2Robo(chassis_r1_world, IMU_YAW / 57.32);

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
// void move_mode5(void)
//{	chassis_t chassis_r1_world;
//	chassis_t chassis_r1_robot;
//
//	wheel_t wheel_r1;
//	chassis_r1_world.Vx = -rc_ctrl.rc.ch[0] * 2;
//	chassis_r1_world.Vy = -rc_ctrl.rc.ch[1] * 2;
//	chassis_r1_world.Vomega = rc_ctrl.rc.ch[2] / 60;
//
//	chassis_r1_robot = Formula_World2Robo(chassis_r1_world, IMU_YAW / 57.32);
//
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
// }

void move_mode_nrf(void)
{
	chassis_t chassis_r1_world;
	chassis_t chassis_r1_robot;

	wheel_t wheel_r1;
	chassis_r1_world.Vx = angol_remote[0] * 6;
	chassis_r1_world.Vy = -angol_remote[1] * 6;
	chassis_r1_world.Vomega = angol_remote[2] / 60;

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

void move_mode_v(int vx, int vy, int vomega)
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
		gm3508_cmd(PID_velocity_realize_chassis_3508(wheel_r1.V[0] * pow(cos(pid_p_chassis_6020[0].error[0] / 1305), 3), 1), PID_velocity_realize_chassis_3508(wheel_r1.V[1] * pow(cos(pid_p_chassis_6020[1].error[0] / 1305), 3), 2), PID_velocity_realize_chassis_3508(wheel_r1.V[2] * pow(cos(pid_p_chassis_6020[2].error[0] / 1305), 3), 3), 0);
	}
}
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

uint8_t move_mode4()
{

	//	if (yaw > 4 || yaw< -4)
	//	{

	chassis_t chassis_r1_world;	            // 世界坐标系
	chassis_t chassis_r1_robot;	            // 机器人坐标系
	wheel_t wheel_r1;	            // 轮系
	chassis_r1_world.Vx = angol_remote[0] * 6;  // position
	chassis_r1_world.Vy = -angol_remote[1] * 6; // position
	chassis_r1_world.Vomega = -PID_position_chassis_omega(0);
	chassis_r1_robot = Formula_World2Robo(chassis_r1_world, yaw / 57.32);
	wheel_r1 = Formula(chassis_r1_robot);
	gm6020_cmd(PID_call_6020(tansform_6020(wheel_r1.theta[0]), 1), PID_call_6020(tansform_6020(wheel_r1.theta[1]), 2), PID_call_6020(tansform_6020(wheel_r1.theta[2]), 3), 0);
	gm3508_cmd(PID_velocity_realize_chassis_3508(wheel_r1.V[0] * cos(pid_p_chassis_6020[0].error[0] / 1305), 1), PID_velocity_realize_chassis_3508(wheel_r1.V[1] * cos(pid_p_chassis_6020[1].error[0] / 1305), 2), PID_velocity_realize_chassis_3508(wheel_r1.V[2] * cos(pid_p_chassis_6020[2].error[0] / 1305), 3), 0);
	return 0;
	//	}

	//	else
	//	{
	//		gm6020_cmd(PID_velocity_realize_6020(0, 1), PID_velocity_realize_6020(0, 2), PID_velocity_realize_6020(0, 3), 0);
	//		gm3508_cmd(PID_velocity_realize_chassis_3508(0, 1), PID_velocity_realize_chassis_3508(0, 2), PID_velocity_realize_chassis_3508(0, 3), 0);

	//		return 1;
	//	}
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

extern int angol_remote[4];
extern uint8_t nrf_key[20];
void move_mode6(void)
{
	chassis_t chassis_r1_world;
	chassis_t chassis_r1_robot;
	int vx, vy, vomga;

	vx = -angol_remote[2] * 2;
	vy = -angol_remote[3] * 2;
	vomga = vx = angol_remote[0];
	wheel_t wheel_r1;
	chassis_r1_world.Vx = vx;
	chassis_r1_world.Vy = vy;
	chassis_r1_world.Vomega = vomga;

	chassis_r1_robot = Formula_World2Robo(chassis_r1_world, 0);

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
void Task_LED(void const *argument)
{
	// 空闲一段时间
	vTaskDelay(CHASSIS_TASK_INIT_TIME); // 上电初始化  马盘初始化等待时间
	// uint8_t flage1 = 40;
	for (;;)
	{
		// HAL_UART_Transmit_DMA( &huart3, tx_buffer, 1);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_11);
		// HAL_UART_Transmit_DMA( &huart3, tx_buffer, 1);
		osDelay(200);
	}
}
uint8_t if_pid_omega_task_finish = 0;
void Pid_omega_task(void const *argument)
{
	// 空闲一段时间

	for (;;)
	{

		move_mode_v(Vx, Vy, PID_call_omega(omega));

		osDelay(6);
	}
}
#define y_offset 75
#define x_offset 80
fp64 theatax, yaw_offset;
int dDistance;
int x1, x2, dx;
extern fp32 yaw_real;
fp64 yaw_offset;
int ttx;
extern osThreadId omega_taskHandle;
int target_x, target_y;
void frist_zone_task(void const *argument)
{
	// 空闲一段时间

	for (;;)
	{

		osDelay(10);
		static uint8_t delay_omega = 0;
		Vx = -2000;
		Vy = 2300;

		vTaskResume(omega_taskHandle);

		osDelay(1200);
		Vx = -1450;
		Vy = 0;
		omega = 90;
		osDelay(900);
		Vx = 0;
		Vy = 0;

		osDelay(1000);

		while (delay_omega != 30)
		{
			target_x = 200;
			target_y = 3130 + y_offset;
			Vx = PID_position_L1S_Y2(target_y);
			Vy = PID_position_L1S_X1(target_x);
			osDelay(10);
			if ((abs((int)gdistance_Y2 - target_y) < 5) && (abs((int)gdistance_X1 - target_x) < 5))
			{
				delay_omega++;
			}
			else
			{

				delay_omega = 0;
			}
		}
		delay_omega = 0;

		// vTaskSuspend(omega_taskHandle);
		while (delay_omega != 30)
		{ // dx bt27
			target_x = 75;
			target_y = 3130 + y_offset;
			Vx = PID_position_L1S_Y2(target_y);
			Vy = PID_position_L1S_X1(target_x);
			osDelay(10);
			if ((abs((int)gdistance_Y2 - target_y) < 4) && (abs((int)gdistance_X1 - target_x) < 4))
			{
				delay_omega++;
			}
			else
			{

				delay_omega = 0;
			}
		}
		delay_omega = 0;
		Vx = 0;
		Vy = 0;
		osDelay(1000);
		// 取苗动作
		Vx = 0;
		Vy = 0;

		Vx = -1500;
		Vy = 1500;
		osDelay(500);
		omega = -90;
		osDelay(1000);

		Vx = 0;
		Vy = 0;

		osDelay(500);

		delay_omega = 0;

		osDelay(10);
//		x1 = gdistance_Y2;
//		x2 = gdistance_Y1;

//		dx = x1 - x2;

//		theatax = atan((double)dx / 376.0) * 180.0 / 3.141;
//		fp32 now_yaw = yaw_real;
//		yaw_offset = -90.0 - theatax - now_yaw;

		osDelay(2000);
		omega = -90;
		osDelay(1000);
		//		while (!(delay_omega == 20))
		//		{
		//			if (fabs(omega - yaw) <2)
		//			{
		//				delay_omega++;
		//			}
		//			else
		//			{
		//				delay_omega = 0;
		//			}
		//			osDelay(6);
		//		}

		delay_omega = 0;

		// atan();

		delay_omega = 0;
		while (delay_omega != 30)
		{
			target_x = 340;
			target_y = 3070;
			Vx = -PID_position_L1S_Y1(target_y);
			Vy = PID_position_L1S_X2(target_x);
			osDelay(10);
			if ((abs((int)gdistance_Y1 - target_y) < 5) && (abs((int)gdistance_X2 - target_x) < 10))
			{
				delay_omega++;
			}
			else
			{

				delay_omega = 0;
			}
		}
		delay_omega = 0;

		// vTaskSuspend(omega_taskHandle);
		while (delay_omega != 20)
		{ // dx bt27
			target_x = 75;
			target_y = 3070;
			Vx = -PID_position_L1S_Y1(target_y);
			Vy = PID_position_L1S_X2(target_x);
			osDelay(10);
			if ((abs((int)gdistance_Y1 - target_y) < 4) && (abs((int)gdistance_X2 - target_x) < 4))
			{
				delay_omega++;
			}
			else
			{

				delay_omega = 0;
			}
		}
		delay_omega = 0;

		Vx = 0;
		Vy = 0;

		/////////////////////////////第二次取苗

		osDelay(1000);
		//////////////////////////////////////////ss

		Vx = 1500;
		Vy = 1500;

		osDelay(1000);

		Vx = 0;
		Vy = 0;

		osDelay(120);
		//		x1=gdistance_Y2;
		//		x2=gdistance_Y1;
		//
		//		dx=x1-x2;
		//
		//
		//		 theatax=atan((double)dx/377.0)*180.0/3.141;
		//		  now_yaw=yaw_real;
		//		yaw_offset=-90.0-theatax-now_yaw;
		//

		delay_omega = 0;

		// vTaskSuspend(omega_taskHandle);
		while (delay_omega != 30)
		{ // dx bt27
			target_x = 1850;
			target_y = 2450;
			Vx = -PID_position_L1S_Y1(target_y);
			Vy = PID_position_L1S_X2(target_x);
			osDelay(10);
			if ((abs((int)gdistance_Y1 - target_y) < 2) && (abs((int)gdistance_X2 - target_x) < 2))
			{
				delay_omega++;
			}
			else
			{

				delay_omega = 0;
			}
		}
		delay_omega = 0;

		Vx = 0;
		Vy = 0;
		// 第一次放苗
		osDelay(1000);
		///////////////////////////////////////

		delay_omega = 0;
		Vx = -1000;
		Vy = -2000;
		osDelay(600);
		Vx = 0;
		Vy = 0;
		osDelay(300);

		//		//vTaskSuspend(omega_taskHandle);
		//				while (delay_omega != 30)
		//		{ //dx bt27
		//			target_x = 1590;
		//			target_y = 2450;
		//			Vx = -PID_position_L1S_Y1(target_y);
		//			Vy = PID_position_L1S_X2(target_x);
		//			osDelay(10);
		//			if ((abs((int)gdistance_Y1 - target_y) < 2)&&(abs((int)gdistance_X2 - target_x) < 2))
		//			{
		//				delay_omega++;
		//			}
		//			else
		//			{

		//				delay_omega = 0;
		//			}
		//		}
		//				delay_omega = 0;

		// vTaskSuspend(omega_taskHandle);
		while (delay_omega != 30)
		{ // dx bt27
			target_x = 1860;
			target_y = 3220;
			Vx = -PID_position_L1S_Y1(target_y);
			Vy = PID_position_L1S_X2(target_x);
			osDelay(10);
			if ((abs((int)gdistance_Y1 - target_y) < 4) && (abs((int)gdistance_X2 - target_x) < 4))
			{
				delay_omega++;
			}
			else
			{

				delay_omega = 0;
			}
		}
		delay_omega = 0;
		// 第二次放苗

		osDelay(800);
		//////////////

		Vx = 0;
		Vy = -2000;
		osDelay(800);
		omega = 90;
		Vy = 0;
		osDelay(1000);

		while (delay_omega != 30)
		{
			target_x = 1377;
			target_y = 3070;
			Vx = PID_position_L1S_Y1(target_y);
			Vy = PID_position_L1S_X1(target_x);
			osDelay(10);
			if ((abs((int)gdistance_Y1 - target_y) < 4) && (abs((int)gdistance_X1 - target_x) < 4))
			{
				delay_omega++;
			}
			else
			{

				delay_omega = 0;
			}
		}
		delay_omega = 0;

		Vx = 0;
		Vy = 0;

		// 第三次放苗
		osDelay(1000);
		/////
		Vx = 1400;
		Vy = -1400;
		osDelay(800);

		while (delay_omega != 30)
		{
			target_x = 1377;
			target_y = 2325;
			Vx = PID_position_L1S_Y1(target_y);
			Vy = PID_position_L1S_X1(target_x);
			osDelay(10);
			if ((abs((int)gdistance_Y1 - target_y) < 2) && (abs((int)gdistance_X1 - target_x) < 2))
			{
				delay_omega++;
			}
			else
			{

				delay_omega = 0;
			}
		}
		delay_omega = 0;
		// 第四次放苗
		osDelay(2000);
		//

		while (delay_omega != 30)
		{
			target_x = 200;
			target_y = 3130 - 1250 + y_offset;
			Vx = PID_position_L1S_Y2(target_y);
			Vy = PID_position_L1S_X1(target_x);
			osDelay(10);
			if ((abs((int)gdistance_Y2 - target_y) < 5) && (abs((int)gdistance_X1 - target_x) < 5))
			{
				delay_omega++;
			}
			else
			{

				delay_omega = 0;
			}
		}
		delay_omega = 0;

		while (delay_omega != 30)
		{
			target_x = 200;
			target_y = 3130 - 1250 + y_offset;
			Vx = PID_position_L1S_Y2(target_y);
			Vy = PID_position_L1S_X1(target_x);
			osDelay(10);
			if ((abs((int)gdistance_Y2 - target_y) < 5) && (abs((int)gdistance_X1 - target_x) < 5))
			{
				delay_omega++;
			}
			else
			{

				delay_omega = 0;
			}
		}
		delay_omega = 0;

		while (delay_omega != 30)
		{
			target_x = 75;
			target_y = 3130 - 1250 + y_offset;
			Vx = PID_position_L1S_Y2(target_y);
			Vy = PID_position_L1S_X1(target_x);
			osDelay(10);
			if ((abs((int)gdistance_Y2 - target_y) < 5) && (abs((int)gdistance_X1 - target_x) < 5))
			{
				delay_omega++;
			}
			else
			{

				delay_omega = 0;
			}
		}
		delay_omega = 0;

		vTaskSuspend(NULL);
		osDelay(6);
	}
}
