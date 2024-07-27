#include "chassis.h"

// #include "chassis_task.h"
// #include "global_position.h"

extern RC_ctrl_t rc_ctrl;
#define PI 3.1415926

/*
*********************************************************************************************************
* Brief    : 世界坐标系转换机器人坐标系
*
* Param(s) : world - velocity vector(Vx,Vy,Vz) in inertia frame. (mm/s, rad/s)
*      robo  - velocity vector(Vx,Vy,Vz) in robot frame. (mm/s, rad/s)
*
* Return(s): none.
*
*********************************************************************************************************
*/
chassis_t Formula_World2Robo(chassis_t world, fp32 theta)
{
        chassis_t robot;
        float sinTheta = sin(theta);
        float cosTheta = cos(theta);

        robot.Vx = world.Vx * cosTheta + world.Vy * sinTheta;
        robot.Vy = world.Vy * cosTheta - world.Vx * sinTheta;
        robot.Vomega = world.Vomega;
        return robot;
}

/*
*********************************************************************************************************
* Brief    : 机器人坐标系转换为轮子的坐标系
*
* Param(s) : wheel - velocity vector(Vx,Vy,Vz) in wheel frame. (0.1 enc counts/s)
*      robo  - velocity vector(Vx,Vy,Vz) in robot frame. (mm/s, rad/s)
*
* Return(s): none.
*·
*********************************************************************************************************
*/
// float theta_angle[3];//转向以角度形式展示
#define Rotation_radius 254 // 旋转半径
wheel_t Formula(chassis_t chassis)
{
        // 处理速度  角度
        // 右手系   轮子id顺时针排序    前轮3508 id为1   前轮2006 id为4

        wheel_t wheel;

        float Vx_part[3] = {0, 0, 0}, Vy_part[3] = {0, 0, 0}; // 三个轮子的x，y速度和角度

        // 平移旋转速度正交叠加
        Vx_part[0] = chassis.Vx + chassis.Vomega * Rotation_radius / 2;
        Vx_part[1] = chassis.Vx - chassis.Vomega * Rotation_radius;
        Vx_part[2] = chassis.Vx + chassis.Vomega * Rotation_radius / 2;
        Vy_part[1] = chassis.Vy;
        Vy_part[2] = chassis.Vy - chassis.Vomega * Rotation_radius * sqrt(3) / 2;
        Vy_part[0] = chassis.Vy + chassis.Vomega * Rotation_radius * sqrt(3) / 2;
        wheel.V[0] = sqrt(Vx_part[0] * Vx_part[0] + Vy_part[0] * Vy_part[0]);
        wheel.V[1] = sqrt(Vx_part[1] * Vx_part[1] + Vy_part[1] * Vy_part[1]);
        wheel.V[2] = sqrt(Vx_part[2] * Vx_part[2] + Vy_part[2] * Vy_part[2]);

        // 计算角度   0---360
        for (int i = 0; i < 3; i++)
        {

                if (Vx_part[i] != 0)
                {
                        if (Vy_part[i] >= 0)
                        {
                                wheel.theta[i] = acos(Vx_part[i] / wheel.V[i]) / PI * 180;
                        }
                        else
                        {
                                wheel.theta[i] = 360 - acos(Vx_part[i] / wheel.V[i]) / PI * 180;
                        }
                }
                else
                {
                        if (Vy_part[i] > 0)
                        {
                                wheel.theta[i] = 90;
                        }
                        else if (Vy_part[i] > 0)
                        {
                                wheel.theta[i] = 90;
                        }
                        else
                        {
                                wheel.theta[i] = -90;
                        }
                }
        }
        return wheel;

} // Formula_4Omni末尾

int16_t tansform_6020(float theta)
{
        int16_t encoder = 0;
        float i = theta / 360;
        encoder = 8191 * (1 - i);
        return encoder;
}


