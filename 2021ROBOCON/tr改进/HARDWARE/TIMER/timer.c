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
extern struct        //����ң���������ṹ�����
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
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///ʹ��TIM3ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	
}

int a = 0 ;


void TIM2_IRQHandler(void)       //��ʱ���жϣ�ֻҪ��⵽ң�˶��ˣ�����Ϳ�ʼ�����жϣ����趨Ŀ�����ٶȵ�ֵ��
{
   static u16 i = 0;
	 if (i>65534)
		 i = 0;
	
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //����ж�
		{
			TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //����жϱ�־λ���Ա�֮������ж�
			
			if(rc.s1==3 && rc.s2==3)          //3,3�ĵ�λ�������м䣩
			{
				Speed_Motor_Target_1 =  0.707106f * rc.ch2 + 0.707106f * rc.ch3;        //��1�ŵ����4��ȫ�����˶����������Ŀ���ٶȣ���s1��ҡ�ˣ�
				Speed_Motor_Target_2 =  0.707106f * rc.ch2 - 0.707106f * rc.ch3;				//��2�ŵ����
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
			 
			
			CAN_CMD_can1_1(PID_velocity_realize_1(Speed_Motor_Target_1-PID_YAW_POS_realize(0),1),         //�����;�PID�㷨�����Ŀ��ֵ��������Ǵ����ֵ���ٶȻ�
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























