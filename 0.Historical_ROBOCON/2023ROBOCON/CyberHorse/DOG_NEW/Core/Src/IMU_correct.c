#include "IMU_correct.h"
#include "CAN_PID_user.h"
#include "mot_move.h"
#include "bsp_rc.h"
#include "postion_clac.h"
#include "pid.h"
#include "math.h"

#define ReductionAndAngleRatio 436.926337f 
#define PI 3.1415926


/*
Yaw--------围绕z轴旋转
Pitch------围绕y_dog轴旋转
Roll-------围绕x轴旋转
*/

extern GaitParams state_gait_params[];
extern float roll,yaw,pitch;
extern float x_dog,y_dog,theta2,theta1;
extern temp_data temp_pid;
float roll_set=0,pitch_set=0;

float yaw_set=0;
float normal_params_l,normal_params_r;
float roll_offset,pitch_offset;
float step_len_initial=14.0f;
extern DetachedParam Gait_Data[];
float step_len_dev;
extern pid_type_def pid_imu;
enum IMU_Use IMU_state;

void IMU_correct_dog(){

if(IMU_state==trot){
	
				//计算陀螺仪yaw轴角度，输出给两腿腿部差速 保持角度
        step_len_dev=PID_calc(&pid_imu,yaw/1.0f,yaw_set/1.0f);  

        float _dev_limit = 2.2;
        if(step_len_dev>_dev_limit)	step_len_dev=_dev_limit;
        else if(step_len_dev<-_dev_limit)	step_len_dev=-_dev_limit;

        normal_params_l=step_len_initial-step_len_dev;
        normal_params_r=step_len_initial+step_len_dev;
        if(normal_params_l<0) normal_params_l=0;
        if(normal_params_r<0) normal_params_r=0;

		Gait_Data[0].detached_params_0.step_length=normal_params_l;
		Gait_Data[0].detached_params_1.step_length=normal_params_r;
		Gait_Data[0].detached_params_2.step_length=normal_params_l;
		Gait_Data[0].detached_params_3.step_length=normal_params_r;



}
	
if(IMU_state==walk){
	
				//计算陀螺仪yaw轴角度，输出给两腿腿部差速 保持角度

        step_len_dev=PID_calc(&pid_imu,yaw/1.0f,yaw_set/1.0f);  

        float _dev_limit = 2.2;
        if(step_len_dev>_dev_limit)	step_len_dev=_dev_limit;
        else if(step_len_dev<-_dev_limit)	step_len_dev=-_dev_limit;

        normal_params_l=step_len_initial-step_len_dev;
        normal_params_r=step_len_initial+step_len_dev;
        if(normal_params_l<0) normal_params_l=0;
        if(normal_params_r<0) normal_params_r=0;


		Gait_Data[2].detached_params_0.step_length=normal_params_l;
		Gait_Data[2].detached_params_1.step_length=normal_params_r;
		Gait_Data[2].detached_params_2.step_length=normal_params_l;
		Gait_Data[2].detached_params_3.step_length=normal_params_r;
	


}

if(pitch!=0||roll!=0){

        roll_offset=15.0f*sin(  PID_calc(&pid_imu,roll*PI/180.0f,roll_set)  );   //roll

        pitch_offset=22.5f*sin(  PID_calc(&pid_imu,-(pitch*PI/180.0f)/2.0f,pitch_set/2.0f ) );  //pitch
				
	
	//CAN1的12，左前一
        x_dog = 0;
        y_dog = state_gait_params[1].stance_height+roll_offset+pitch_offset;
        if(y_dog>29.5f)	y_dog=29.5f;
        else if(y_dog<10.5f)	y_dog=10.5f;
        CartesianToTheta(1.0);
        temp_pid.ref_agle[1]=-theta1*ReductionAndAngleRatio;
        temp_pid.ref_agle[0]=-theta2*ReductionAndAngleRatio;
	
	//CAN1的34，左前2
        x_dog = 0;
        y_dog = state_gait_params[1].stance_height-roll_offset+pitch_offset;
        if(y_dog>29.5f)	y_dog=29.5f;
        else if(y_dog<10.5f)	y_dog=10.5f;
        CartesianToTheta(1.0);
        temp_pid.ref_agle[2]=theta1*ReductionAndAngleRatio;
        temp_pid.ref_agle[3]=theta2*ReductionAndAngleRatio;
				
	//CAN2的12，左后一
        x_dog = 0;
        y_dog = state_gait_params[1].stance_height+roll_offset-pitch_offset;
        if(y_dog>29.5f)	y_dog=29.5f;
        else if(y_dog<10.5f)	y_dog=10.5f;
        CartesianToTheta(1.0);
        temp_pid.ref_agle[5]=-theta1*ReductionAndAngleRatio;
        temp_pid.ref_agle[4]=-theta2*ReductionAndAngleRatio;
				
	//CAN2的34，右后二
        x_dog = 0;
        y_dog = state_gait_params[1].stance_height-roll_offset-pitch_offset;
        if(y_dog>29.5f)	y_dog=29.5f;
        else if(y_dog<10.5f)	y_dog=10.5f;
        CartesianToTheta(1.0);
        temp_pid.ref_agle[6]=theta1*ReductionAndAngleRatio;
        temp_pid.ref_agle[7]=theta2*ReductionAndAngleRatio;
		
}




}




