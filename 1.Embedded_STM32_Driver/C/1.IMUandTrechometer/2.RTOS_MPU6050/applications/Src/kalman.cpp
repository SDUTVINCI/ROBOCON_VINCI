#include "kalman.h"



//输入参数：
//float angle_m 加速度计计算的角度，float gyro_m陀螺仪角速度
//float *angle_f融合后的角度，float *angle_dot_f融合后的角速度


//输出参数：
//滤波后的角度及角速度(float *angle_f融合后的角度，float *angle_dot_f融合后的角速度)

//在滤波融合算法设计过程中，主要对协方差Q和R的取值进行设计（Q_angle, Q_gyro, R_angle），
//R取值越小，滤波响应和收敛越迅速；Q取值越小，抑制滤除噪声的能力越强。
//因此，具体取值也需要反复实际调试进行权衡确定。dt是采样周期。


void kalman_filter(float angle_m, float gyro_m, float *angle_f, float *angle_dot_f)		
{
    //------------------------------
    static float angle, angle_dot; 		
    const float Q_angle = 0.000001, Q_gyro = 0.0001, R_angle = 0.5, dt = 0.002;			
    static float P[2][2]={
                       { 1, 0 },
                       { 0, 1 }
                     };	
    static float Pdot[4] = {0, 0, 0, 0};
    const uint8_t C_0 = 1;
    static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
    //------------------------------
    angle += (gyro_m - q_bias) * dt;

    Pdot[0]  =Q_angle - P[0][1] - P[1][0];
    Pdot[1] = -P[1][1];
    Pdot[2] = -P[1][1];
    Pdot[3] = Q_gyro;

    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;

    angle_err = angle_m - angle;

    PCt_0=C_0 * P[0][0];
    PCt_1=C_0 * P[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
        
    angle	+= K_0 * angle_err;
    q_bias += K_1 * angle_err;
    angle_dot = gyro_m - q_bias;

    *angle_f = angle;
    *angle_dot_f = angle_dot;
}
