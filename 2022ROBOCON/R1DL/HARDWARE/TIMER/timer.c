/**    《遥控器舵轮控制》   
 *           _____                    _____                          
 *          /\    \                  /\    \         
 *         /::\    \                /::\    \        
 *        /::::\    \              /::::\    \       
 *       /::::::\    \            /::::::\    \      
 *      /:::/\:::\    \          /:::/\:::\    \     
 *     /:::/__\:::\    \        /:::/  \:::\    \    
 *    /::::\   \:::\    \      /:::/    \:::\    \   
 *   /::::::\   \:::\    \    /:::/    / \:::\    \  
 *  /:::/\:::\   \:::\____\  /:::/    /   \:::\    \ 
 * /:::/  \:::\   \:::|    |/:::/____/     \:::\____\
 * \::/   |::::\  /:::|____|\:::\    \      \::/    /
 *  \/____|:::::\/:::/    /  \:::\    \      \/____/ 
 *        |:::::::::/    /    \:::\    \    
 *        |::|\::::/    /      \:::\    \   
 *        |::| \::/____/        \:::\    \    
 *        |::|  ~|               \:::\    \    
 *        |::|   |                \:::\    \         
 *        \::|   |                 \:::\____\        
 *         \:|   |                  \::/    /        
 *          \|___|                   \/____/   
 */ 
#include "timer.h"
#include "can.h"
#include "pid_user.h"
#include "usart.h"
#include "math.h"
#include "duolun.h"

#define ABS(x)	((x>0) ? (x) : (-x)) //x只能是数字，不能是表达式

float Speed_Motor_Target_5,Speed_Motor_Target_6,Speed_Motor_Target_7,Speed_Motor_Target_8;//底盘3508目标速度
float moca_motor_target_z,moca_motor_target_y;//摩擦轮电机速度
float angle_zs_target,angle_cp_target;//抓手电机角度，餐盘转向电机角度
float res=0.00f,prev_res=0.00f,Angle_Duo_Target_1,Angle_Duo_Target_2,Angle_Duo_Target_3,Angle_Duo_Target_4,Angle_Duo_Target_Yaw;//底盘2006目标角度
float round_cnt=0.00f,res1,direction_coefficient=1.00f;
float quan1,quan2;

extern struct 
 {
 int16_t ch0;
 int16_t ch1;
 int16_t ch2;
 int16_t ch3;
 int16_t ch4; 
 int8_t s1;
 int8_t s2;
 }rc;



void TIM2_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM2,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}

void TIM2_IRQHandler(void)
	
{
   static u16 i = 0;
	 if (i>65534)
		 i = 0;
	
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //溢出中断
		{
			TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //清除中断标志位
			  {	
/*------------------------------------------------------------------------------------------------------------------*/					
				if(rc.s1==3 && rc.s2==3){		//底盘正常模式
		      res = calc_angle_duolun(rc.ch2,rc.ch3);//计算遥控器遥杆角度		
		      round_cnt = calc_motor_quanshu((res+res1),prev_res);//计算圈数
		
					res1 = calc_min_angle(res,prev_res);  //就近转圈的目标角度
					quan1 = fabs(prev_res-res1);
					quan2 = fabs(res-prev_res);	
          if( quan1 >= quan2 ){
						res1 = 0.00f;
						direction_coefficient = 1.00f;
					}											
				  if(quan1 < quan2){
						res = 0.00f;
						direction_coefficient = -1.00f;
					}
					
       prev_res = res+res1;//记录上一次的角度数据					
			 Angle_Duo_Target_1 = 67.00f/20.00f*(36.00f * 8192.00f * (round_cnt + (res+res1)/360.00f));//将数据转换成轮子转动的角度数据
			 Angle_Duo_Target_2 = 67.00f/20.00f*(36.00f * 8192.00f * (round_cnt + (res+res1)/360.00f));
			 Angle_Duo_Target_3 = 67.00f/20.00f*(36.00f * 8192.00f * (round_cnt + (res+res1)/360.00f));
			 Angle_Duo_Target_4 = 67.00f/20.00f*(36.00f * 8192.00f * (round_cnt + (res+res1)/360.00f));
					
            if(ABS(rc.ch2)>250||ABS(rc.ch3)>250){
				      Speed_Motor_Target_5 =  direction_coefficient *(-0.849106f *  sqrt(rc.ch2*rc.ch2+rc.ch3*rc.ch3));
				      Speed_Motor_Target_6 =  direction_coefficient *(-0.849106f *  sqrt(rc.ch2*rc.ch2+rc.ch3*rc.ch3)-rc.ch3/98.636f);
				      Speed_Motor_Target_7 =  direction_coefficient *(-0.849106f *  sqrt(rc.ch2*rc.ch2+rc.ch3*rc.ch3)-rc.ch3/98.636f);
				      Speed_Motor_Target_8 =  direction_coefficient *(-0.849106f *  sqrt(rc.ch2*rc.ch2+rc.ch3*rc.ch3));
								
              Speed_Motor_Target_5 *= 7;
				      Speed_Motor_Target_6 *= 7;
				      Speed_Motor_Target_7 *= 7;
				      Speed_Motor_Target_8 *= 7;
						}
						if(ABS(rc.ch2)<=250 && ABS(rc.ch3)<=250){
							Speed_Motor_Target_5 = 0;
				      Speed_Motor_Target_6 = 0;
				      Speed_Motor_Target_7 = 0;
				      Speed_Motor_Target_8 = 0;
						}
							angle_zs_target = -0.707106f * rc.ch1;                                                        
				 
		  if(ABS(rc.ch0) >= 200){
//						 Angle_Duo_Target_Yaw = 123494.400f;//自旋模式下舵轮的初始角度
			       Angle_Duo_Target_1 = -123494.400f + 67.00f/20.00f * round_cnt * 8192.00f * 36.00f;
			       Angle_Duo_Target_2 =  123494.400f + 67.00f/20.00f * round_cnt * 8192.00f * 36.00f;
			       Angle_Duo_Target_3 = -123494.400f + 67.00f/20.00f * round_cnt * 8192.00f * 36.00f;
			       Angle_Duo_Target_4 =  123494.400f + 67.00f/20.00f * round_cnt * 8192.00f * 36.00f;}
			if(ABS(rc.ch0) >= 330){
				Speed_Motor_Target_5 = +0.707106f*rc.ch0;// 原地旋转
				Speed_Motor_Target_6 = -0.707106f*rc.ch0;
				Speed_Motor_Target_7 = -0.707106f*rc.ch0;
				Speed_Motor_Target_8 = +0.707106f*rc.ch0;
				
				Speed_Motor_Target_5 *= 4.2f;
				Speed_Motor_Target_6 *= 4.2f;
				Speed_Motor_Target_7 *= 4.2f;
				Speed_Motor_Target_8 *= 4.2f;
			}
			if(ABS(rc.ch0)<330)
				 Angle_Duo_Target_Yaw = 0.000f;
	
				
					 
		  CAN_CMD_can1_1(pid_call_1(Angle_Duo_Target_1-Angle_Duo_Target_Yaw,1), //向电机发送数据
										 pid_call_1(Angle_Duo_Target_2+Angle_Duo_Target_Yaw,2),
			               pid_call_1(Angle_Duo_Target_3-Angle_Duo_Target_Yaw,3),
		                 pid_call_1(Angle_Duo_Target_4+Angle_Duo_Target_Yaw,4));	
		
			CAN_CMD_can1_2(PID_velocity_realize_1(Speed_Motor_Target_5,5),
										 PID_velocity_realize_1(Speed_Motor_Target_6,6), 
										 PID_velocity_realize_1(Speed_Motor_Target_7,7),
										 PID_velocity_realize_1(Speed_Motor_Target_8,8));
			CAN_CMD_can2_1(PID_velocity_realize_2(angle_zs_target,1),
										 PID_velocity_realize_2(0,2),
										 PID_velocity_realize_2(0,3),
										 PID_velocity_realize_2(0,4));		 
						 }

/*------------------------------------------------------------------------------------------------------------------*/			 
		if(rc.s1==1 && rc.s2==3){//摩擦轮模式		 餐盘转向  
			Angle_Duo_Target_Yaw = 123494.400f;
			if(rc.ch1>100){
			  moca_motor_target_z = +2.056277f * rc.ch1;//摩擦轮电机速度
				moca_motor_target_y = -2.056277f * rc.ch1;			
				moca_motor_target_z *= 7;
				moca_motor_target_y *= 7;}
			if(rc.ch1 ==0)
			{
				moca_motor_target_z = 0;
				moca_motor_target_y = 0;
				angle_zs_target = 0;
			}
			if(rc.ch1 == -660)
				angle_zs_target = -0.707106f * rc.ch1;
			if(ABS(rc.ch2)==660)
				angle_cp_target = 0.0906f * rc.ch2;//餐盘转向电机角度
			if(rc.ch2==0)
				angle_cp_target=0;
			CAN_CMD_can1_1(pid_call_1(Angle_Duo_Target_1+Angle_Duo_Target_Yaw,1), //向电机发送数据
										 pid_call_1(Angle_Duo_Target_2-Angle_Duo_Target_Yaw,2),
			               pid_call_1(Angle_Duo_Target_3+Angle_Duo_Target_Yaw,3),
		                 pid_call_1(Angle_Duo_Target_4-Angle_Duo_Target_Yaw,4));	
		
			CAN_CMD_can1_2(PID_velocity_realize_1(0,5),
										 PID_velocity_realize_1(0,6), 
										 PID_velocity_realize_1(0,7),
										 PID_velocity_realize_1(0,8));
		  CAN_CMD_can2_1(PID_velocity_realize_2(angle_zs_target,1),
										 PID_velocity_realize_2(angle_cp_target,2),
										 PID_velocity_realize_2(moca_motor_target_z,3),
										 PID_velocity_realize_2(moca_motor_target_y,4));	
					}
		if(rc.s1 !=  1){
			Angle_Duo_Target_Yaw = 0.00f;
			CAN_CMD_can1_1(pid_call_1(Angle_Duo_Target_1+Angle_Duo_Target_Yaw,1), //向电机发送数据
										 pid_call_1(Angle_Duo_Target_2-Angle_Duo_Target_Yaw,2),
			               pid_call_1(Angle_Duo_Target_3+Angle_Duo_Target_Yaw,3),
		                 pid_call_1(Angle_Duo_Target_4-Angle_Duo_Target_Yaw,4));			
		}
/*------------------------------------------------------------------------------------------------------------------*/			
		if(rc.s1==2 && rc.s2==3){
			    res = calc_angle_duolun(rc.ch2,rc.ch3);//计算遥控器遥杆角度		
		      round_cnt = calc_motor_quanshu((res+res1),prev_res);//计算圈数
		
					res1 = calc_min_angle(res,prev_res);  //就近转圈的目标角度
					quan1 = fabs(prev_res-res1);
					quan2 = fabs(res-prev_res);
//					prev_res_last = prev_res;		
          if( quan1 >= quan2 ){
						res1 = 0.00f;
						direction_coefficient = 1.00f;
					}											
				  if(quan1 < quan2){
						res = 0.00f;
						direction_coefficient = -1.00f;
					}
       prev_res = res;//记录上一次的角度数据					
			 Angle_Duo_Target_1 = 67.00f/20.00f*(36.00f * 8192.00f * (round_cnt + (res+res1)/360.00f));//将数据转换成轮子转动的角度数据
			 Angle_Duo_Target_2 = 67.00f/20.00f*(36.00f * 8192.00f * (round_cnt + (res+res1)/360.00f));
			 Angle_Duo_Target_3 = 67.00f/20.00f*(36.00f * 8192.00f * (round_cnt + (res+res1)/360.00f));
			 Angle_Duo_Target_4 = 67.00f/20.00f*(36.00f * 8192.00f * (round_cnt + (res+res1)/360.00f));
					

				      Speed_Motor_Target_5 =  direction_coefficient *(-0.849106f *  sqrt(rc.ch2*rc.ch2+rc.ch3*rc.ch3));
				      Speed_Motor_Target_6 =  direction_coefficient *(-0.849106f *  sqrt(rc.ch2*rc.ch2+rc.ch3*rc.ch3)-rc.ch3/88.236f);
				      Speed_Motor_Target_7 =  direction_coefficient *(-0.849106f *  sqrt(rc.ch2*rc.ch2+rc.ch3*rc.ch3)-rc.ch3/88.236f);
				      Speed_Motor_Target_8 =  direction_coefficient *(-0.849106f *  sqrt(rc.ch2*rc.ch2+rc.ch3*rc.ch3));
								
              Speed_Motor_Target_5 *= 7;
				      Speed_Motor_Target_6 *= 7;
				      Speed_Motor_Target_7 *= 7;
				      Speed_Motor_Target_8 *= 7;                                                       
						 
		  if(ABS(rc.ch0) >= 300){
//						 Angle_Duo_Target_Yaw = 123494.400f;//自旋模式下舵轮的初始角度
			       Angle_Duo_Target_1 = -123494.400f + round_cnt * 1.00f;
			       Angle_Duo_Target_2 =  123494.400f + round_cnt * 1.00f;
			       Angle_Duo_Target_3 = -123494.400f + round_cnt * 1.00f;
			       Angle_Duo_Target_4 =  123494.400f + round_cnt * 1.00f;}
			if(ABS(rc.ch0) >= 500){
				Speed_Motor_Target_5 = +0.707106f*rc.ch0;// 原地旋转
				Speed_Motor_Target_6 = -0.707106f*rc.ch0;
				Speed_Motor_Target_7 = -0.707106f*rc.ch0;
				Speed_Motor_Target_8 = +0.707106f*rc.ch0;
				
				Speed_Motor_Target_5 *= 2.1f;
				Speed_Motor_Target_6 *= 2.1f;
				Speed_Motor_Target_7 *= 2.1f;
				Speed_Motor_Target_8 *= 2.1f;
			}
			if(ABS(rc.ch0)<300)
				 Angle_Duo_Target_Yaw = 0.000f;
					 
		  CAN_CMD_can1_1(pid_call_1(Angle_Duo_Target_1-Angle_Duo_Target_Yaw,1), //向电机发送数据
										 pid_call_1(Angle_Duo_Target_2+Angle_Duo_Target_Yaw,2),
			               pid_call_1(Angle_Duo_Target_3-Angle_Duo_Target_Yaw,3),
		                 pid_call_1(Angle_Duo_Target_4+Angle_Duo_Target_Yaw,4));	
		
			CAN_CMD_can1_2(PID_velocity_realize_1(Speed_Motor_Target_5,5),
										 PID_velocity_realize_1(Speed_Motor_Target_6,6), 
										 PID_velocity_realize_1(Speed_Motor_Target_7,7),
										 PID_velocity_realize_1(Speed_Motor_Target_8,8));
			CAN_CMD_can2_1(pid_call_2(angle_zs_target,1),
										 PID_velocity_realize_2(0,2),
										 PID_velocity_realize_2(0,3),
										 PID_velocity_realize_2(0,4));	
		      }	
		
		   }
				
		 }
		
		
	 }

 
	 
	 
	 

	 
	 
