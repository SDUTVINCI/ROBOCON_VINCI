/***      《 舵轮角度计算 》

 *      ┌─┐       ┌─┐ + +
 *   ┌──┘ ┴───────┘ ┴──┐++
 *   │                 │
 *   │       ───       │++ + + +
 *   ███████───███████│+
 *   │                 │+
 *   │      ─┴─        │
 *   │                 │
 *   └───┐         ┌───┘
 *       │         │
 *       │         │   + +
 *       │         │
 *       │         └──────────────┐
 *       │                        │
 *       │                        ├─┐
 *       │                        ┌─┘
 *       │                        │
 *       └─┐  ┐  ┌───────┬──┐  ┌──┘  + + + +
 *         │ ─┤ ─┤       │ ─┤ ─┤
 *         └──┴──┘       └──┴──┘  + + + +
 *
 *
 */
#include "helm_wheel.h"

//计算遥控器遥杆角度
float calc_angle_helm_wheel(float set_ch2, float set_ch3)
{
	if (ABS(set_ch2) > 120 || ABS(set_ch3) > 120)
		return (atan2(-set_ch2, set_ch3)) * 180.00f / 3.14159f; // atan2函数math.h
	else
		return 0.00f;
}

//计算电机应该转动的圈数，防止跳变
float calc_motor_round_cnt(float angle, float last_angle)
{
	static float round_cnt = 0.00f;
	if (angle - last_angle > 260) //记录轮子转动圈数
		round_cnt--;
	else if (angle - last_angle < -260)
		round_cnt++;
	return round_cnt;
}

//优化舵轮转动角度，就近原则
float calc_min_angle(float set_angle, float last_set_angle)
{
	float target_angle;
	if (fabs(set_angle - last_set_angle) > 90)
	{
		if (set_angle >= 90 && set_angle <= 181)
		{
			target_angle = set_angle - 180.00f;
		}
		if (set_angle <= -90 && set_angle >= -181)
		{
			target_angle = set_angle + 180.00f;
		}
	}
	else
	{
		target_angle = set_angle;
	}
	return target_angle;
}
