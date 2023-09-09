#include "timer.h"
#include "can.h"
#include "pid_user.h"
#include "usart.h"
#include "math.h"
#include "control.h"

float shensuo=0;
float set_yaw=0.0f;
extern float yaw;
float Speed_Motor_Target_1,Speed_Motor_Target_2,Speed_Motor_Target_3,Speed_Motor_Target_4;
float set_x, set_y;
float v_x,v_y,v_yaw;
float xiebo_jd,zhua_jd;
int Speed=300;
extern struct        //定义遥控器按键结构体变量
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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	
}

int a = 0 ;


void TIM2_IRQHandler(void)       //定时器中断，只要检测到遥杆动了，立马就开始发生中断，并设定目标电机速度的值。
{
   static u16 i = 0;
	 if (i>65534)
		 i = 0;
	
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //溢出中断
		{
			TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //清除中断标志位，以便之后结束中断
			
			if(rc.s1==3 && rc.s2==3)          //3,3的档位（都在中间）
			{
				Speed_Motor_Target_1 =  0.707106f * rc.ch2 + 0.707106f * rc.ch3;        //（1号电机）4轮全向轮运动解析，电机目标速度（用s1左摇杆）
				Speed_Motor_Target_2 =  0.707106f * rc.ch2 - 0.707106f * rc.ch3;				//（2号电机）
				Speed_Motor_Target_3 = -0.707106f * rc.ch2 - 0.707106f * rc.ch3;
				Speed_Motor_Target_4 = -0.707106f * rc.ch2 + 0.707106f * rc.ch3;
				
				Speed_Motor_Target_1 *= 7;
				Speed_Motor_Target_2 *= 7;
				Speed_Motor_Target_3 *= 7;
				Speed_Motor_Target_4 *= 7;
			}
			if(rc.s1==2 && rc.s2==3)
			{
				Speed_Motor_Target_1 =  0.707106f * rc.ch2 + 0.707106f * rc.ch3;
				Speed_Motor_Target_2 =  0.707106f * rc.ch2 - 0.707106f * rc.ch3;
				Speed_Motor_Target_3 = -0.707106f * rc.ch2 - 0.707106f * rc.ch3;
				Speed_Motor_Target_4 = -0.707106f * rc.ch2 + 0.707106f * rc.ch3;
			}
			if(rc.s1==1 && rc.s2==3)
			{
				Speed_Motor_Target_1 = 0;
				Speed_Motor_Target_2 = 0;
				Speed_Motor_Target_3 = 0;
				Speed_Motor_Target_4 = 0;
			}
			
//			if(rc.s2==1)
//			{
//				Speed_Motor_Target_1 =  0;
//				Speed_Motor_Target_2 =  0;
//				Speed_Motor_Target_3 =  0;
//				Speed_Motor_Target_4 = 	0;
//			}
//			
			if(rc.s2==2)
			{
				Speed_Motor_Target_1 =  0.707106f * rc.ch2 + 0.707106f * rc.ch3;
				Speed_Motor_Target_2 =  0.707106f * rc.ch2 - 0.707106f * rc.ch3;
				Speed_Motor_Target_3 = -0.707106f * rc.ch2 - 0.707106f * rc.ch3;
				Speed_Motor_Target_4 = -0.707106f * rc.ch2 + 0.707106f * rc.ch3;
			}
			if(rc.s2==1)
			 {	 
					//if (i%2==0) 
					{
						 if (a==1)
						 {
							 if (Jugement_position(a_1[0],a_1[1],a_1[2])==0)
							 {
									Car_position_control(a_1[0],a_1[1],a_1[2]);	 
							 }
							 else
								  {
									 Speed_Motor_Target_1 = 0;
									 Speed_Motor_Target_2 = 0;
									 Speed_Motor_Target_3 = 0;
									 Speed_Motor_Target_4 = 0;
							 
									}
						 }
						 else if (a==2)
						 {
							 if (Jugement_position(a_2[0],a_2[1],a_2[2])==0)
							 {
									Car_position_control(a_2[0],a_2[1],a_2[2]);
							 }
							 else
								 {
									 Speed_Motor_Target_1 = 0;
									 Speed_Motor_Target_2 = 0;
									 Speed_Motor_Target_3 = 0;
									 Speed_Motor_Target_4 = 0;
							 
								}
						 }
						 else if (a==3)
						 {
							 if (Jugement_position(a_3[0],a_3[1],a_3[2])==0)
							 {
									Car_position_control(a_3[0],a_3[1],a_3[2]);	
							 }
							 else
								  {
									 Speed_Motor_Target_1 = 0;
									 Speed_Motor_Target_2 = 0;
									 Speed_Motor_Target_3 = 0;
									 Speed_Motor_Target_4 = 0;
							 
								}
						 }					 
						 else if (a==4)
						 {
							 if (Jugement_position(a_4[0],a_4[1],a_4[2])==0)
							 {
									Car_position_control(a_4[0],a_4[1],a_4[2]);	
							 }
							 else
								 {
									 Speed_Motor_Target_1 = 0;
									 Speed_Motor_Target_2 = 0;
									 Speed_Motor_Target_3 = 0;
									 Speed_Motor_Target_4 = 0;
							 
								}
						 }
						 else if (a==5)
						 {
							 if (Jugement_position(a_5[0],a_5[1],a_5[2])==0)
							 {
									Car_position_control(a_5[0],a_5[1],a_5[2]);	
							 }
							 else
							 {
									 Speed_Motor_Target_1 = 0;
									 Speed_Motor_Target_2 = 0;
									 Speed_Motor_Target_3 = 0;
									 Speed_Motor_Target_4 = 0;
							 
							 }
						 }
				 }
			}
			 
			
			CAN_CMD_can1_1(PID_velocity_realize_1(Speed_Motor_Target_1-PID_YAW_POS_realize(0),1),         //立马发送经PID算法处理的目标值结合陀螺仪处理的值的速度环
											PID_velocity_realize_1(Speed_Motor_Target_2-PID_YAW_POS_realize(0),2), 
												PID_velocity_realize_1(Speed_Motor_Target_3-PID_YAW_POS_realize(0),3),
													PID_velocity_realize_1(Speed_Motor_Target_4-PID_YAW_POS_realize(0),4));
			CAN_CMD_can1_2(0,
											pid_call_1(angle_motor_t_3508(-zhua_jd),6), 
												pid_call_1(angle_motor_t_3508(zhua_jd),7),
																0);
			CAN_CMD_can2_1(pid_call_2(angle_motor_t_2006(-360*shensuo),1),
											pid_call_2(0,2),
												pid_call_2(angle_motor_t_xiebo(xiebo_jd),3),
													0);
		
		}
}























