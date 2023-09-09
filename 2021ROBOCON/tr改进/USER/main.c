#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "timer.h"
#include "can.h"
#include "pid_user.h"
#include "pid.h"
#include "pwm.h"
#include "gpio.h"


char 	make_zero[5] = {0xff, 0xaa, 0x76, 0x00, 0x00};
char unlocking[5] = {0xff, 0xaa, 0x69, 0x88, 0xb5};
//extern float yaw;
extern float  zangle;
extern float xiebo_jd,zhua_jd;
extern float shensuo;
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

 float ccc = 1.0f;
 
 extern int a;

int main(void)

{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(168);
	CAN1_mode_init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);
	CAN2_mode_init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);
	pid_chassis_init();
	uart_init(115200);
	TIM2_Int_Init(50,8400-1);
	TIM_PWM_Init(19999,167);
	gpio_Init_u();
  delay_ms(100);
	
	
//ÍÓÂÝÒÇ½âËø
//	for(int t=0;t<5;t++)
//	{
//		USART_SendData(USART1, unlocking[t]);         
//		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
//	}
////ÍÓÂÝÒÇ¹éÁã
//		for(int t=0;t<5;t++)
//	{
//		USART_SendData(USART1, make_zero[t]);         
//		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
//	}
//	
	
  while(1)
	{
		static u16 i = 0;
		i++;
		if (i>65534)
	  i = 0;
		if(rc.s1==3 && rc.s2==3)
		{
			if(rc.ch1==660)
			{
				xiebo_jd = 0;
				shensuo = 0;//33
			}
			if(rc.ch1==-660)
			{
				shensuo = 33;//33
			}
//			set_yaw += rc.ch0/8000.0f;
		
		}
		if(rc.s1==2 && rc.s2==3)
		{
			if(rc.ch0==660)
			{
				TIM_SetCompare1(TIM4,2200);
				TIM_SetCompare2(TIM4,2200);
				TIM_SetCompare3(TIM4,2200);
				TIM_SetCompare4(TIM4,2200);
				TIM_SetCompare1(TIM1,2200);
				
			}
			if(rc.ch0==-660)
			{
				TIM_SetCompare1(TIM4,1100);
				TIM_SetCompare2(TIM4,1100);
				TIM_SetCompare3(TIM4,1100);
				TIM_SetCompare4(TIM4,1100);
				TIM_SetCompare1(TIM1,1100);
			}
				if(rc.ch1==660)
			{
				zhua_jd = 95; 
				delay_ms(500);
				xiebo_jd = 0;
				delay_ms(2000);
				TIM_SetCompare1(TIM4,1500);
				TIM_SetCompare2(TIM4,1500);
				TIM_SetCompare3(TIM4,1500);
				TIM_SetCompare4(TIM4,1500);
				TIM_SetCompare1(TIM1,1500);
				delay_ms(500);
				shensuo = 22;
				delay_ms(2000);
				TIM_SetCompare1(TIM4,2200);
				TIM_SetCompare2(TIM4,2200);
				TIM_SetCompare3(TIM4,2200);
				TIM_SetCompare4(TIM4,2200);
				TIM_SetCompare1(TIM1,2200);
				xiebo_jd = 115;//112
			}
			if(rc.ch1==-660)
			{
				zhua_jd = 0;
				xiebo_jd = 0;
				shensuo = 33 ;
			}	
		}
		if(rc.s1==1 && rc.s2==3)
		{
			if(rc.ch3==660)
			{
				PEout(6) = 1;//1
				delay_ms(500);
				PEout(6) = 0;
				delay_ms(500);	
			}
			if(rc.ch3==-660)
			{
				PFout(1) = 1;//2
				delay_ms(500);
				PFout(1) = 0;
				delay_ms(500);
			}
			if(rc.ch2==-660)
			{
				PCout(13) = 1;//3
				delay_ms(500);
				PCout(13) = 0;
				delay_ms(500);
			
			}
			if(rc.ch2==660)
			{
				PCout(15) = 1;//4
				delay_ms(500);
				PCout(15) = 0;
				delay_ms(500);
			}
			if(rc.ch1==660)
			{
				PCout(14) = 1;//5
				delay_ms(500);
				PCout(14) = 0;
				delay_ms(500);
			}
		}
		if(rc.s2==2)
		{
			 if(rc.ch1==660)
			 {
				shensuo = 33;
				zhua_jd = 0;	 
			 }
			 if(rc.ch0==-660)
			 {
					TIM_SetCompare1(TIM4,1200);
					TIM_SetCompare2(TIM4,1200);
					TIM_SetCompare3(TIM4,1200);
					TIM_SetCompare4(TIM4,1200);
					TIM_SetCompare1(TIM1,1200);
			 
			 }
			 if(rc.ch1==-660)
			 {
					zhua_jd = 95;	
					delay_ms(2000);
					TIM_SetCompare1(TIM4,1500);
					TIM_SetCompare2(TIM4,1500);
					TIM_SetCompare3(TIM4,1500);
					TIM_SetCompare4(TIM4,1500);
					TIM_SetCompare1(TIM1,1500);
			 }
				if(rc.ch0==660)
				{
					xiebo_jd = 0;
					delay_ms(2500);
					shensuo = 22;
					delay_ms(2000);
					xiebo_jd = 115;		
					TIM_SetCompare1(TIM4,2200);
					TIM_SetCompare2(TIM4,2200);
					TIM_SetCompare3(TIM4,2200);
					TIM_SetCompare4(TIM4,2200);
					TIM_SetCompare1(TIM1,2200);					
				}
		}
		
		delay_ms(5);
//		if(i%100)
//		{
//			if (fabs(yaw - zangle)>ccc)
//			{
//				Update_yaw(yaw);
//				
//	//			Update_y(1000.0);
//	//			delay_ms(10);
//	//			Update_x(1000.0);
//	//			delay_ms(10);
//				//ccc = 0;
//			}
//		}
		
		if(rc.s2==1)
		{
			if (rc.ch3 == 660) // 
				a = 1;
			else if (rc.ch3 == -660)
				a = 2;
			else if (rc.ch2 == -660)
				a = 3;
			else if (rc.ch2 == 660)
				a = 4;
			else if (rc.ch1 == 660)
				a = 5;
		}
		
		
		
		
	}
}
