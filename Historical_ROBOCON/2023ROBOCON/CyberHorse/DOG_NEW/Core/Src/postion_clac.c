#include "postion_clac.h"
#include "can.h"
#include "pid.h"
#include "math.h"
#include "main.h"
#include "CAN_PID_User.h"
#include "stdbool.h"

#define L1 8
#define L2 16
#define L3 16
#define L4 8
#define L5 10
#define L6 5
#define arefa 0.001

/*  腿部平行时  13fd3421.856f  */
#define PI 3.1415926
float x_dog=0,y_dog=0,theta2,theta1;
temp_data temp_pid= {0};
float now_time;
#define ReductionAndAngleRatio 436.926337f
               //157293/360 电机转子转过一定角度所需脉冲数
#define true 1


float gp;
uint16_t _leg_active[]={0,0,0,0};
extern uint16_t Time_count;
float _climbing_offset_angle=15;
extern motor_measure_t motor_can1[8];
extern motor_measure_t motor_can2[8];
 uint16_t really_star_gait;
 uint16_t flag;

static float oldOutData = 0.0f;
double nowData,get_Data,nowOutData;
float filter(float Data )
{
	
    nowData  = Data; 
    nowOutData = arefa * nowData  + (1.0f - arefa) * oldOutData;
    oldOutData = nowOutData;
    return nowOutData;  
}

/**
*	最低walk {12.0, 15.0, 1.8, 0.00, 0.50, 1.0}
*	效果不错{14.0, 12.0, 3.5, 0.00, 0.50, 1.0},
*	{16.0, 12.00, 5.0, 0.00, 0.20, 2.0}, // WALK		ahead 不错
*	{17.3, 12.00, 4.0, 0.00, 0.35, 2.5}, // WALK		ahead
* 对角小跑步态不错速度和稳定都非常好
*/

/*
参数要改
*/
GaitParams state_gait_params[]={
	    //{身高, 步长, 抬腿高, 压腿高, 飞行占比, 频率}	单位 cm
				{14,  0.0  ,12.0,     2,    0.2,    1.8},
				{16.0, 12.00, 5.0, 0.40, 0.25, 3.6},//walk
				{17.0, 0.00, 6.00, 4.00, 0.35, 2.0}, 	//BOUND
				{27.3, 12.0 ,20.0 ,2.00, 0.30, 1.0},
				{17.3,12.0,4.0,0.00,0.25,1},					//原地转

};
//typedef struct {
//    float stance_height;// 步行时身体与地面的期望高度 *Desired height of body from ground during walking (m)
//    float step_length ;// 全步长(cm)*Peak amplitude below stanceHeight in sinusoidal trajectory (m)
//    float up_amp; // 在正弦轨迹中，足部峰值在支架高度以上(m)*Height the foot peaks at above the stanceHeight in sinusoidal trajectory (m)
//    float down_amp;// 正弦轨迹低于stanceHeight的峰值振幅(m)*Portion of the gait time should be doing the down portion of trajectory
//    float flight_percent;//飞行相时间占比 *Length of entire step (m)
//    float freq;//一个步态周期的频率(Hz)*Frequency of one gait cycle (Hz)
//}GaitParams;

DetachedParam Gait_Data[]={
		  //{身高,   步长, 抬腿高, 压腿高, 飞行占比, 频率}	单位 cm
{
				{16,  3.0  ,2.5,     2,    0.2,    1.8},
				{16,  3.0  ,2.5,     2,    0.2,    1.8},
				{16,  3.0 , 2.5,     2,    0.2,    1.8},
				{16,  3.0 , 2.5,     2,    0.2,    1.8},
},//trot1  0

	
{
				{17.0, 4.0, 6.0,   0.0,    0.3,   1.5},
				{17.0, 4.0, 6.0,   0.0,    0.3,   1.5},
				{17.0, 4.0, 6.0,   0.0,    0.3,   1.5},
				{17.0, 4.0, 6.0,   0.0,    0.3,   1.5},
},			//walk   1

{
				{16.0, 6.0, 2.5,   2,    0.2,   1.6},
				{16.0, 2.0, 2.5, 2, 0.2, 1.6},
				{16.0, 2.0, 2.5, 2, 0.2, 1.6},
				{16.0, 6.0, 2.5, 2, 0.2, 1.6},
},//Zuo_Zhuan   2

{
				{16.0, 2.0, 2.5,   2,    0.2,   1.8},
				{16.0, 6.0, 2.5, 2, 0.2, 1.8},
				{16.0, 6.0, 2.5, 2, 0.2, 1.8},
				{16.0, 2.0, 2.5, 2, 0.2, 1.8},
},//You_Zhuan  3   hao xiang bi zhixing zhi

{
				{16,  4.0  ,2.5,     2,    0.2,    1.8},
				{16,  4.0  ,2.5,     2,    0.2,    1.8},
				{16,  4.0 , 2.5,     2,    0.2,    1.8},
				{16,  4.0 , 2.5,     2,    0.2,    1.8},
},	//duo jiao   4
	

{
				{16,  1.6  ,6.8,     2,    0.3,    2.0},
				{16,  1.6  ,6.8,     2,    0.3,    2.0},
				{16,  1.6 , 6.8,     2,    0.3,    2.0},
				{16,  1.6 , 6.8,     2,    0.3,    2.0},
},//yuan di xuan zhuan 5

{
				{14.0,  6.8 , 2.1,     6.4,    0.4,    1.2},
				{14.0,  6.8,  2.1,     6.4,    0.4,    1.2},
				{14.0,  6.8 , 2.1,     6.4,    0.4,    1.2},
				{14.0,  6.8 , 2.1,     6.4,    0.4,    1.2},
},//TROT  2use   6

{
				{14,  5.0  ,0,     2,    0.8,    2.0},
				{14,  5.0  ,0,     2,    0.8,    2.0},
				{14,  5.0 , 0,     2,    0.8,    2.0},
				{14,  5.0 , 0,     2,    0.8,    2.0},
},//Xie Po Ce Shi  7



{
				{14.0, 2.0, 3.0,   0,    0.8,   1.6},
				{14.0, 2.0, 3.0, 0, 0.8, 1.6},
				{14.0, 2.0, 3.0, 0, 0.8, 1.6},
				{14.0, 2.0, 3.0, 0, 0.8, 1.6},
},//Xie Po Zou Shang Qu  8

{
				{14,  2.0  ,10,     0,    0.8,    1.8},
				{14,  2.0  ,10,     0,    0.8,    1.8},
				{17,  2.0 , 10,     0,    0.8,    1.8},
				{17,  2.0 , 10,     0,    0.8,    1.8},
},//TROT shang lou Ti   9

{
				{16,  4.0  ,2.5,     2,    0.3,    1.5},
				{16,  4.0  ,2.5,     2,    0.3,    1.5},
				{16,  4.0 , 2.5,     2,    0.3,    1.5},
				{16,  4.0 , 2.5,     2,    0.3,    1.5},
},//back   10

{
				{16,  1.8  ,2.5,     2,    0.3,    2.2},
				{16,  1.8  ,2.5,     2,    0.3,    2.2},
				{16,  1.8 , 2.5,     2,    0.3,    2.2},
				{16,  1.8, 2.5,     2,    0.3,    2.2},
},   //11

{
				{16,  1  ,3.5,     2,    0.3,    2.4},
				{16,  1  ,3.5,     2,    0.3,    2.4},
				{16,  1 , 3.5,     2,    0.3,    2.4},
				{16,  1 , 3.5,     2,    0.3,    2.4},
},//yuan di xuan zhuan 12

{
				{16,  1.0  ,3.5,     4,    0.3,    2.4},
				{16,  1.0  ,3.5,     4,    0.3,    2.4},
				{16,  1.0 , 3.5,     4,    0.3,    2.4},
				{16,  1.0 , 3.5,     4,    0.3,    2.4},
},//yuan di xuan zhuan 13

{
				{16,  2.0  ,2.5,     2,    0.2,    1.8},
				{16,  4.0  ,2.5,     2,    0.2,    1.8},
				{16,  4.0 , 2.5,     2,    0.2,    1.8},
				{16,  2.0 , 2.5,     2,    0.2,    1.8},
},//  14

{
				{16,  4.0  ,2.5,     2,    0.2,    1.8},
				{16,  2.0  ,2.5,     2,    0.2,    1.8},
				{16,  2.0 , 2.5,     2,    0.2,    1.8},
				{16,  4.0 , 2.5,     2,    0.2,    1.8},
},//  15

};

void gait_detached( GaitParams d_params,
                     float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,
                     float leg0_direction, float leg1_direction,float leg2_direction, float leg3_direction) {

    float t = HAL_GetTick()/1000.0f-now_time/1000.0f;
											
    // const float leg0_direction = 1.0;
											 
			//CAN1的12电机
    if(_leg_active[0]==1)
        CoupledMoveLeg( t, d_params, leg0_offset, leg0_direction, 0);

    // const float leg1_direction = 1.0;
		
		//CAN1的34电机
    if(_leg_active[1]==1)
        CoupledMoveLeg( t, d_params, leg1_offset, leg1_direction, 1);

    // const float leg2_direction = 1.0;
		
		//CAN2的12电机
    if(_leg_active[2]==1)
        CoupledMoveLeg( t, d_params, leg2_offset, leg2_direction, 2);

    //  const float leg3_direction = 1.0;
		//CAN2的34电机
    if(_leg_active[3]==1)
        CoupledMoveLeg( t, d_params, leg3_offset, leg3_direction, 3);
		
}


//  	gait(Gait_Data[0], 0.5, 0, 0, 0.5, 1.0, 1.0, 1.0, 1.0);
void gait(	DetachedParam params,
            float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,
            float leg0_direction, float leg1_direction,float leg2_direction, float leg3_direction) {


    float t = HAL_GetTick()/1000.0f-now_time/1000.0f;

    //printf("\r\n t=%f",t);

    // const float leg0_direction = 1.0;
    CoupledMoveLeg( t, params.detached_params_0, leg0_offset, leg0_direction, 0);

    // const float leg1_direction = 1.0;
    CoupledMoveLeg( t, params.detached_params_1, leg1_offset, leg1_direction, 1);

    // const float leg2_direction = 1.0;
    CoupledMoveLeg( t, params.detached_params_2, leg2_offset, leg2_direction, 2);

    //  const float leg3_direction = 1.0;
    CoupledMoveLeg( t, params.detached_params_3, leg3_offset, leg3_direction, 3);

    //????PD
    // ChangeTheGainsOfPD(gains);
}

void CoupledMoveLeg(float t, GaitParams params,float gait_offset, float leg_direction, int LegId)
{		

    SinTrajectory(t, params, gait_offset);		//?????????????
    CartesianToTheta(leg_direction);		//?????????????????????
    SetCoupledPosition(LegId);		//????????????????????
	
}

void SinTrajectory (float t,GaitParams params, float gaitOffset) {
    static float p = 0;
    static float prev_t = 0;

    float stanceHeight = params.stance_height;  //步行时身体与地面的期望高度
    float downAMP =filter( params.down_amp);  //正弦轨迹低于stanceHeight的峰值振幅(m)
    float upAMP =filter( params.up_amp);     //在正弦轨迹中，足部峰值在支架高度以上(m)
    float flightPercent = params.flight_percent;//飞行相时间占比
    float stepLength =filter (params.step_length); //全步长(cm)
    float FREQ = params.freq; //一个步态周期的频率(Hz)
	
    p += FREQ * (t - prev_t < 0.5 ? t - prev_t : 0); // ????????????? should reduce the lurching when starting a new gait
    prev_t = t;
	
    float gp = fmod((p+gaitOffset),1.0);
    if (gp <= flightPercent) {
        x_dog = (gp/flightPercent)*stepLength - stepLength/2.0f;
        y_dog = -upAMP*sin(PI*gp/flightPercent) + stanceHeight;
    }
    else {
        float percentBack = (gp-flightPercent)/(1.0f-flightPercent);
        x_dog = -percentBack*stepLength + stepLength/2.0f;
        y_dog = downAMP*sin(PI*percentBack) + stanceHeight;
    }
		
		if(gp>0.9f){
		flag++;
			if(flag==1)
		really_star_gait=1;
		}

}

void CartesianToTheta(float leg_direction)
{
	    float L=0;
    float N=0;
    double M=0;
    float A1=0;
    float A2=0;

    L=sqrt(		pow(x_dog,2)	+		pow(y_dog,2)	);

    if(L<5) L=5.856f;
    else if(L>23.7f) L=23.7f;

    // vTaskSuspend(MotorControlTask_Handler);

    N=asin(x_dog/L)*180.0/PI;
    M=acos(	(pow(L,2)+pow(L1,2)-pow(L2,2))/(2*L1*L)	)*180.0/PI;
    A1=M-N;

    A2=M+N;

    if(leg_direction==1.0) {
        theta2=(A1-70.0f);
        theta1=(A2-70.0f);
    } else if(leg_direction==-1.0) {
        theta1=(A1-70.0f);
        theta2=(A2-70.0f);
    }


}
//qian tui
float x1_dog,y1_dog;
float x2_dog,y2_dog;

void CartesianToTheta1(float leg_direction)
{
    float L=0;
    float N=0;
    double M=0;
    float A1=0;
    float A2=0;

    L=sqrt(	pow(x1_dog,2) + pow(y1_dog,2) );
	
    if(L<5) L=5.856f;
    else if(L>23.7f) L=23.7f;

    N=asin(x1_dog/L)*180.0f/PI;
    M=acos(	(pow(L,2)+pow(L1,2)-pow(L2,2))/(2*L1*L)	)*180.0f/PI;
    A1=M-N;

    A2=M+N;

    if(leg_direction==1.0f) {
        theta2=(A1-70.0f);
        theta1=(A2-70.0f);
    } 
		else if(leg_direction==-1.0f) {
        theta1=(A1-70.0f);
        theta2=(A2-70.0f);
    }

		
}

void CartesianToTheta2(float leg_direction)
{
    float L=0;
    float N=0;
    double M=0;
    float A1=0;
    float A2=0;

    L=sqrt(	pow(x2_dog,2) + pow(y2_dog,2) );
	
    if(L<5) L=5.856f;
    else if(L>23.7f) L=23.7f;

    N=asin(x2_dog/L)*180.0f/PI;
    M=acos(	(pow(L,2)+pow(L1,2)-pow(L2,2))/(2*L1*L)	)*180.0f/PI;
    A1=M-N;

    A2=M+N;

    if(leg_direction==1.0f) {
        theta2=(A1-70.0f);
        theta1=(A2-70.0f);
    } 
		else if(leg_direction==-1.0f) {
        theta1=(A1-70.0f);
        theta2=(A2-70.0f);
    }
}

uint8_t _climbing_offset_flag;

void SetCoupledPosition( int LegId)
{
	//CAN1的12电机
	if(_climbing_offset_flag==1){
        if(LegId==0)
        {
            temp_pid.ref_agle[1]=-theta1*ReductionAndAngleRatio+_climbing_offset_angle;
            temp_pid.ref_agle[0]=-theta2*ReductionAndAngleRatio-_climbing_offset_angle;
        }
	//CAN1的34电机

        else if(LegId==1)
        {
            temp_pid.ref_agle[2]=theta1*ReductionAndAngleRatio-_climbing_offset_angle;
            temp_pid.ref_agle[3]=theta2*ReductionAndAngleRatio+_climbing_offset_angle;
        }
	//CAN2的12电机

        else if(LegId==2)
        {
            temp_pid.ref_agle[5]=-theta1*ReductionAndAngleRatio+_climbing_offset_angle;
            temp_pid.ref_agle[4]=-theta2*ReductionAndAngleRatio-_climbing_offset_angle;
        }
		//CAN2的34电机

        else if(LegId==3)
        {
            temp_pid.ref_agle[6]=+theta1*ReductionAndAngleRatio-_climbing_offset_angle;
            temp_pid.ref_agle[7]=+theta2*ReductionAndAngleRatio	+_climbing_offset_angle;
        }
			}
				
    else {
        if(LegId==0)
        {
            temp_pid.ref_agle[1]=-theta1*ReductionAndAngleRatio;
            temp_pid.ref_agle[0]=-theta2*ReductionAndAngleRatio;
        }
        else if(LegId==1)
        {
            temp_pid.ref_agle[2]=theta1*ReductionAndAngleRatio;
            temp_pid.ref_agle[3]=theta2*ReductionAndAngleRatio;
        }

        else if(LegId==2)
        {
            temp_pid.ref_agle[5]=-theta1*ReductionAndAngleRatio;
            temp_pid.ref_agle[4]=-theta2*ReductionAndAngleRatio;
        }
        else if(LegId==3)
        {
            temp_pid.ref_agle[6]=+theta1*ReductionAndAngleRatio;
            temp_pid.ref_agle[7]=+theta2*ReductionAndAngleRatio;
        }
    	
}
		
}

void SetPosition( int LegId){
//1shun 2ni 3shun 4ni
 if(LegId==0)
        {
            temp_pid.ref_agle[1]=-theta1*ReductionAndAngleRatio;
            temp_pid.ref_agle[0]=theta2*ReductionAndAngleRatio;
        }
        else if(LegId==1)
        {
            temp_pid.ref_agle[2]=theta1*ReductionAndAngleRatio;
            temp_pid.ref_agle[3]=-theta2*ReductionAndAngleRatio;
        }

        else if(LegId==2)
        {
            temp_pid.ref_agle[5]=-theta1*ReductionAndAngleRatio;
            temp_pid.ref_agle[4]=theta2*ReductionAndAngleRatio;
        }
        else if(LegId==3)
        {
            temp_pid.ref_agle[6]=theta1*ReductionAndAngleRatio;
            temp_pid.ref_agle[7]=-theta2*ReductionAndAngleRatio;
        }



}




float pos_agle[8];


 
 bool IsValidGaitParams( GaitParams params) {
    const float maxL = 24.0f;
    const float minL = 8.0f;

    float stanceHeight = params.stance_height;
    float downAMP = params.down_amp;
    float upAMP = params.up_amp;
    float flightPercent = params.flight_percent;
    float stepLength = params.step_length;
    float FREQ = params.freq;

    if (stanceHeight + downAMP > maxL || sqrt(pow(stanceHeight, 2) + pow(stepLength / 2.0f, 2)) > maxL) {
        return false;
    }
    if (stanceHeight - upAMP < minL) {

        return false;
    }

    if (flightPercent <= 0 || flightPercent > 1.0) {
 
        return false;
    }

    if (FREQ < 0) {
 
        return false;
    }

    if (FREQ > 10.0f) {
;
        return false;
    }

    return true;
}
 
 void MoveLegs()
{
 for(uint8_t i=0; i<8; i++){
 pos_agle[i]=temp_pid.ref_agle[i];
 }
 
		CAN1_CMD_1(pid_call_1(pos_agle[0],1),
			pid_call_1(pos_agle[1],2),
			pid_call_1(pos_agle[2],3),
			pid_call_1(pos_agle[3],4));

		CAN2_CMD_1(pid_call_2(pos_agle[4],1),
			pid_call_2(pos_agle[5],2),
			pid_call_2(pos_agle[6],3),
			pid_call_2(pos_agle[7],4));

 }
 