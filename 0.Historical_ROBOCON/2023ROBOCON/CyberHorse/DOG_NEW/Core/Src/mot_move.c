#include "mot_move.h"
#include "CAN_PID_User.h"
#include "can.h"
#include "pid.h"
#include "tim.h"
#include "postion_clac.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "IMU_correct.h"
#include "math.h"
#include "string.h"

#define ReductionAndAngleRatio 436.926337  //3591/187*8191/360=436.926337
#define PI 3.1415926

extern enum Gati Pid_Correspondence;
extern int min_output,max_i_output,angle_min_output,angle_i_max_output;
extern uint16_t _leg_active[];
extern float x_dog,y_dog;
extern DetachedParam Gait_Data[];
extern GaitParams state_gait_params[];
uint8_t jump_flag;
extern motor_measure_t motor_can1[8];
extern motor_measure_t motor_can2[8];
extern PID_Data CAN1_PID_DATA[2][4];
extern PID_Data CAN2_PID_DATA[2][4];
extern uint16_t pid_data_use;
extern enum IMU_Use IMU_state;

extern RC_ctrl_t rc_ctrl;
extern float yaw_set;
extern float yaw;
uint8_t mot_clear;
extern temp_data temp_pid;

uint8_t Pos_Star;
float ref_star_agle[8]= {0};
uint16_t pos_star_angle[8]={0};
uint16_t cl;
extern uint16_t really_star_gait;
extern uint16_t flag;
void Tort_gait(){
	
	if(really_star_gait==0){
				 IMU_state=trot;
			 pid_data_use=0;
		   min_output=8000,max_i_output=8000;
		 angle_min_output=1000,angle_i_max_output=1000;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[0], 0.5, 0, 0, 0.5, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
	
	}

	if(really_star_gait==1){
			 IMU_state=trot;
			 pid_data_use=0;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[0], 0.5, 0, 0, 0.5, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
		flag=0;
		return;
	}
	
	
}

void Walk_gait(){
		if(really_star_gait==0){
				 IMU_state=trot;
			 pid_data_use=0;
		   min_output=2000,max_i_output=2000;
		 angle_min_output=800,angle_i_max_output=800;
			 pid_chassis_init();
//	  IMU_correct_dog();
	gait(Gait_Data[1], 0.25, 0.75, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
	
	}
				if(really_star_gait==1){

//			IMU_state=walk;
		 pid_data_use=0;
		 min_output=10000,  max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
		pid_chassis_init();
//	IMU_correct_dog();
	gait(Gait_Data[1], 0.25, 0.75, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
}

}

//zuo zhuan
void Lift_gait_Xuan_Zhuan(){
				if(really_star_gait==0){
				 IMU_state=trot;
			 pid_data_use=0;
		   min_output=8000,max_i_output=8000;
		 angle_min_output=1000,angle_i_max_output=1000;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[12], 0.0, 0.5, 0.5, 0.00, -1.0, 1.0, 1.0, -1.0);
		MoveLegs();
	
	}
if(really_star_gait){
	
				pid_data_use=0;
		 min_output=10000,  max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
	 pid_chassis_init();
	
   gait(Gait_Data[12], 0.0, 0.5, 0.5, 0.00, -1.0, 1.0, 1.0, -1.0); //zuo zhuan
		MoveLegs();
}
	
}


//zuo zhuan
void Lift_Zhuan(){

			if(really_star_gait==0){
	

				IMU_state=trot;
			 pid_data_use=0;
		   min_output=8000,max_i_output=8000;
		 angle_min_output=1000,angle_i_max_output=1000;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[15], 0.0, 0.5, 0.5, 0.00, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
	
	}
if(really_star_gait){
	
				pid_data_use=0;
		 min_output=9000,  max_i_output=9000;
		 angle_min_output=3000,angle_i_max_output=3000;
	 pid_chassis_init();
	
   gait(Gait_Data[15], 0.0, 0.5, 0.5, 0.00, 1.0, 1.0, 1.0, 1.0); //zuo zhuan
		MoveLegs();
}
}


//you zhuan
void Right_zhuan(){

			if(really_star_gait==0){
				 IMU_state=trot;
			 pid_data_use=0;
		   min_output=8000,max_i_output=8000;
		 angle_min_output=1000,angle_i_max_output=1000;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[14], 0.0, 0.5, 0.5, 0.00, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
	
	}
if(really_star_gait){
	
				pid_data_use=0;
		 min_output=10000,  max_i_output=10000;
		 angle_min_output=6000,angle_i_max_output=6000;
	 pid_chassis_init();
	
  	gait(Gait_Data[14], 0.0, 0.5, 0.5, 0.00, 1.0, 1.0, 1.0, 1.0); 
		MoveLegs();
}
	
}

//zuo zhuan
void Right_gait_Xuan_Zhuan(){
				if(really_star_gait==0){
				 IMU_state=trot;
			 pid_data_use=0;
		   min_output=8000,max_i_output=8000;
		 angle_min_output=1000,angle_i_max_output=1000;
			 pid_chassis_init();
//	  IMU_correct_dog();
   gait(Gait_Data[13], 0.0, 0.5, 0.5, 0.00, 1.0, -1.0, -1.0, 1.0); //you zhuan
		MoveLegs();
	
	}

	if(really_star_gait==1){
				pid_data_use=0;
		 min_output=10000,  max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
	 pid_chassis_init();
	
   gait(Gait_Data[13], 0.0, 0.5, 0.5, 0.00, 1.0, -1.0, -1.0, 1.0); //you zhuan
		MoveLegs();
	}
}

void Back_gait(){
				if(really_star_gait==0){
				 IMU_state=trot;
			 pid_data_use=0;
		   min_output=2000,max_i_output=2000;
		 angle_min_output=800,angle_i_max_output=800;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[10], 0.5, 0, 0, 0.5, -1.0, -1.0, -1.0, -1.0);
		MoveLegs();
	
	}
								if(really_star_gait==1){

			 IMU_state=trot;
			 pid_data_use=0;
		   min_output=9000,max_i_output=9000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
	  IMU_correct_dog();
  	gait(Gait_Data[10], 0.5, 0, 0, 0.5, -1.0, -1.0, -1.0, -1.0);
		MoveLegs();

}
}
    const float stance_height = 24.0f; 

extern float x1_dog,y1_dog,x2_dog,y2_dog;
void JUMP_Star(){

			 pid_data_use=0;
		   min_output=12000,max_i_output=12000;
		 angle_min_output=3000,angle_i_max_output=3000;
			 pid_chassis_init();
	
	////jump_star x y 
//      x_dog = 3.0;
//			y_dog = 8.8;
//			CartesianToTheta(-1.0);
//	for(uint8_t m=0;m<4;m++){
//	SetPosition(m);
//	}
//	
//			MoveLegs();
//	
	     x_dog = stance_height*sin(10*PI/180);
			y_dog = stance_height*cos(10*PI/180);
	CartesianToTheta(1.0);
	for(uint8_t m=0;m<2;m++){
	SetPosition(m);
	}
	
	     x_dog = stance_height*sin(10*PI/180);
			y_dog = stance_height*cos(10*PI/180);
	CartesianToTheta(1.0);
	for(uint8_t m=2;m<4;m++){
	SetPosition(m);
	}

				MoveLegs();

}

void JUMP_Middle1(){


   				pid_data_use=3;

		  min_output=16000,max_i_output=16000;
			angle_min_output=16000,angle_i_max_output=16000;
				pid_chassis_init();
	
	     x1_dog = 6.0;
			y1_dog = 15.8;
	CartesianToTheta1(-1.0);
	for(uint8_t m=0;m<2;m++){
	SetPosition(m);
	}
	
	x2_dog=6.0;
	y2_dog=14.8;
	CartesianToTheta2(-1.0);
	for(uint8_t m=2;m<4;m++){
	SetPosition(m);
	}
	

		MoveLegs();

	return;
	
}

void JUMP_Middle2(){


   				pid_data_use=3;

		  min_output=16000,max_i_output=16000;
			angle_min_output=16000,angle_i_max_output=16000;
				pid_chassis_init();
	
      x1_dog = 6.0;
			y1_dog = 14.8;
	CartesianToTheta1(1.0);
	for(uint8_t m=0;m<2;m++){
	SetPosition(m);
	}
	
	x2_dog=6.0;
	y2_dog=15.8;
	CartesianToTheta2(1.0);
	for(uint8_t m=2;m<4;m++){
	SetPosition(m);
	}
	
	CartesianToTheta1(1);
	CartesianToTheta2(1);
	for(uint8_t m=0;m<4;m++){
	SetPosition(m);
	}
	
			MoveLegs();
	return;
	
}


void JUMP_End1(){
				pid_data_use=1;
		   min_output=16000,max_i_output=16000;
		 angle_min_output=16000,angle_i_max_output=16000;
				pid_chassis_init();
		   x_dog = 9.0f;
			y_dog = 8.4f;
		  CartesianToTheta(-1.0);
			for(uint8_t m=0;m<4;m++){
			SetPosition(m);
			}
	
					MoveLegs();

}

void JUMP_End2(){

				pid_data_use=8;
		   min_output=1000,max_i_output=1000;
		 angle_min_output=800,angle_i_max_output=800;
				pid_chassis_init();
	      x_dog =9.0f;
			y_dog = 8.4;
		    CartesianToTheta(-1.0);
			for(uint8_t m=0;m<4;m++){
			SetPosition(m);
			}
	
					MoveLegs();
}

float t1;
//修改y的值，改变其距离
void Jump_To_Front1(){
    const float prep_time = 0.5f; // 准备时间，此时腿由初状态变成压缩状态 [s]		0.8
    const float jump_time = 0.20f ; // 伸腿爆发起跳的时间 [s]		0.2
	  const float interval_time=0.45f;
	  float t;

   	static float start_time_;
/*t1从0开始计时，每次计数间隔1s左右	*/
	    t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // 跳跃开始后的时间
			t1+=t-start_time_;
		  start_time_=t;

	if(jump_flag==0)
			t1=0;
				
	    if(t1<prep_time){
		 	  JUMP_Star();
			return;
				}
			
		else if(t1 >= prep_time && t1 < prep_time + jump_time){
			if(rc_ctrl.rc.ch[1]==600){
			JUMP_Middle1();
			return;
				}
			}
		
		else if(t1 >=  prep_time + jump_time && t1 < prep_time + jump_time+interval_time){
			JUMP_End1();
			return;
				}
		
		JUMP_End2();
					return;


	}
void Jump_To_Front2(){
    const float prep_time = 0.5f; // 准备时间，此时腿由初状态变成压缩状态 [s]		0.8
    const float jump_time = 0.10f ; // 伸腿爆发起跳的时间 [s]		0.2
	  const float interval_time=0.45f;
	  float t;

   	static float start_time_;
/*t1从0开始计时，每次计数间隔1s左右	*/
	    t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // 跳跃开始后的时间
			t1+=t-start_time_;
		  start_time_=t;

	if(jump_flag==0)
			t1=0;
				
	    if(t1<prep_time){
		 	  JUMP_Star();
			return;
				}
			
		else if(t1 >= prep_time && t1 < prep_time + jump_time){
			if(rc_ctrl.rc.ch[1]==600){
			JUMP_Middle2();}
			return;
				}
				
		else if(t1 >=  prep_time + jump_time && t1 < prep_time + jump_time+interval_time){
			JUMP_End1();
			return;
				}
		JUMP_End2();
					return;

}

extern uint8_t _climbing_offset_flag;

void Xie_Po(){
					if(really_star_gait==0){
				 IMU_state=trot;
			 pid_data_use=0;
		   min_output=9000,max_i_output=9000;
		 angle_min_output=1000,angle_i_max_output=1000;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[7], 0.5, 0, 0, 0.5, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
	
	}
					
					if(really_star_gait==1){

			 pid_data_use=0;
		   min_output=15000,max_i_output=15000;
		 angle_min_output=5000,angle_i_max_output=5000;
			 pid_chassis_init();
  	gait(Gait_Data[7], 0.5, 0, 0, 0.5, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();

return;
}
					}

void Xie_Po_Zou(){
					if(really_star_gait==0){
				 IMU_state=trot;
			 pid_data_use=0;
		   min_output=9000,max_i_output=9000;
		 angle_min_output=1000,angle_i_max_output=1000;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[8], 0.5, 0, 0, 0.5, 1.0, -1.0, -1.0, 1.0);
		MoveLegs();
	
	}
					if(really_star_gait==1){
			 pid_data_use=0;
		   min_output=13000,max_i_output=12000;
		 angle_min_output=5000,angle_i_max_output=5000;
			 pid_chassis_init();
  	gait(Gait_Data[8], 0.5, 0, 0, 0.5, 1.0, -1.0, -1.0, 1.0);
		MoveLegs();
return;
}
}

void Xie_Po_You(){
					if(really_star_gait==0){
				 IMU_state=trot;
			 pid_data_use=0;
		   min_output=9000,max_i_output=9000;
		 angle_min_output=4000,angle_i_max_output=4000;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[8], 0.5, 0, 0, 0.5, -1.0, 1.0, 1.0, -1.0);
		MoveLegs();
	
	}
					
						if(really_star_gait==1){

			 pid_data_use=0;
		   min_output=13000,max_i_output=13000;
		 angle_min_output=4000,angle_i_max_output=4000;
			 pid_chassis_init();
  	gait(Gait_Data[8], 0.5, 0, 0, 0.5, -1.0, 1.0, 1.0, -1.0);
		MoveLegs();
return;
}
}



void Stop_gait(){

		  	pid_data_use=0;
		   min_output=10000,max_i_output=10000;
		 angle_min_output=3000,angle_i_max_output=3000;
				pid_chassis_init();
	
//	CAN1_CMD_1(pid_call_1(-0*437,1),
//			pid_call_1(0*437,2),
//			pid_call_1(-0*437,3),
//			pid_call_1(0*437,4));

//		CAN2_CMD_1(pid_call_2(0,1),
//			pid_call_2(0,2),
//			pid_call_2(0,3),
//			pid_call_2(0,4));
	x_dog=0;
	y_dog=17.0;
	
		    CartesianToTheta(-1.0);
			for(uint8_t m=0;m<4;m++){
			SetPosition(m);
			}
	
					MoveLegs();
}

void Tort_gait_qiaoqiao(){
					if(really_star_gait==0){
				 IMU_state=trot;
			 pid_data_use=0;
		   min_output=3000,max_i_output=3000;
		 angle_min_output=800,angle_i_max_output=800;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[9], 0.5, 0, 0, 0.5, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
	
	}
						if(really_star_gait==1){

			 IMU_state=trot;
			 pid_data_use=0;
		   min_output=12000,max_i_output=12000;
		 angle_min_output=2000,angle_i_max_output=2000;
			 pid_chassis_init();
//	  IMU_correct_dog();
  	gait(Gait_Data[9], 0.5, 0, 0, 0.5, 1.0, 1.0, 1.0, 1.0);
		MoveLegs();
						}

}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim==&htim8){
				  	pid_data_use=0;
		   min_output=8000,max_i_output=8000;
		 angle_min_output=3000,angle_i_max_output=3000;
				pid_chassis_init();
		cl++;
		
		CAN1_CMD_1(pid_call_1(-21.0f*437,1),
			pid_call_1(21.0f*437,2),
			pid_call_1(-21.0f*437,3),
			pid_call_1(21.0f*437,4));

		CAN2_CMD_1(pid_call_2(-20.0f*437,1),
			pid_call_2(20.0f*437,2),
			pid_call_2(-20.0f*437,3),
			pid_call_2(20.0f*437,4));
		
 if(cl==2000){
	 
	 	     memset(&motor_can1,0,sizeof(motor_measure_t)*4);
	 	     memset(&motor_can2,0,sizeof(motor_measure_t)*4);
    for(int i=0; i<8; i++)
   temp_pid.ref_agle[i]=ref_star_agle[i]=0;
 	 HAL_TIM_Base_Stop_IT(&htim8);
	 HAL_TIM_Base_Stop(&htim8);  //guan bi ding shi qi
 

	}
}
	/*******************************************************************************************************/
	
	if(htim==&htim1){
		//zuo 1   you  0
		if(cl==2000){
//					Walk_gait();

				if(rc_ctrl.rc.s[0]==3&&rc_ctrl.rc.s[1]==3){
				//往前行走
			if(rc_ctrl.rc.ch[3]>500){
			 Tort_gait();
				return;
			}
			
			if(rc_ctrl.rc.ch[3]==-660){
			Back_gait();
			return;
			}
				//Zuo_Zhuan			


			if(rc_ctrl.rc.ch[0]>500){
			Xie_Po_You();
			return;
			}
			
			if(rc_ctrl.rc.ch[0]<-500){
			Xie_Po_Zou();
				return;
			}
			

			if(rc_ctrl.rc.ch[1]==660){
							Walk_gait();
							jump_flag=1;
							return;
			}
			
			if(rc_ctrl.rc.ch[1]==-660){
			      Xie_Po();
			     return;
			}
		
//					if(rc_ctrl.rc.ch[1]<-200){
//						Tort_gait_qiaoqiao();
//						return;
//						}

						if(rc_ctrl.rc.ch[2]>600){
						Right_zhuan();
						return;
						}
						if(rc_ctrl.rc.ch[2]==-660){
						Lift_Zhuan();
						return;
						}
						
						
			if(rc_ctrl.rc.ch[4]==-660){
							Lift_gait_Xuan_Zhuan();
							yaw_set=yaw;

				return;
			}
			  //You_Zhuan
			if(rc_ctrl.rc.ch[4]==660){
							Right_gait_Xuan_Zhuan();
								yaw_set=yaw;
				return;
			}
			flag=0;
			really_star_gait=0;
			Stop_gait();
			jump_flag=0;
			_leg_active[0]=0,_leg_active[1]=0,_leg_active[2]=0,_leg_active[3]=0;
							return;
		}
		
		
		
		
		
		/********hua  li  de  fen  ge xian********/
		
				if(rc_ctrl.rc.s[0]==1&&rc_ctrl.rc.s[1]==3){
					
			if(rc_ctrl.rc.ch[3]>500){
			Walk_gait();	
				return;
			}
				//Zuo_Zhuan			
			if(rc_ctrl.rc.ch[2]==-660){
							Lift_gait_Xuan_Zhuan();
				return;
			}
			  //You_Zhuan
			if(rc_ctrl.rc.ch[2]>500){
							Right_gait_Xuan_Zhuan();
								yaw_set=yaw;
				return;
			}
						
			if(rc_ctrl.rc.ch[3]<-500){
						Back_gait();
				return;
				}
			
				//Xie  Po
			if(rc_ctrl.rc.ch[1]==660){
							Xie_Po();
							return;
			}
							//Xie  Po
			if(rc_ctrl.rc.ch[0]<-600){
							Xie_Po_Zou();
							return;
			}
						if(rc_ctrl.rc.ch[0]>600){
							Xie_Po_You();
							return;
			}
						if(rc_ctrl.rc.ch[1]==-660){
						Tort_gait_qiaoqiao();
						return;
						}
			flag=0;
			really_star_gait=0;
			Stop_gait();
			jump_flag=0;
			_leg_active[0]=0,_leg_active[1]=0,_leg_active[2]=0,_leg_active[3]=0;

							return;
						
				}
		
		
		}
		}
	
	}

	
	
	