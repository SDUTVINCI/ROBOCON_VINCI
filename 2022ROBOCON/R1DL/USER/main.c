#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "timer.h"
#include "can.h"
#include "pid_user.h"
#include "pid.h"
#include "pwm.h"
#include "gpio.h"

extern float angle_zs_target;//抓手电机角度数据
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
int main(void)

  {
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(168);	
	CAN1_mode_init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);
	CAN2_mode_init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);
	pid_chassis_init();
	pid_other_init();
	uart_init(115200);
	Yaw_Init();//陀螺仪初始化 
	TIM2_Int_Init(50,8400-1);
	TIM_PWM_Init(19999,167);
	gpio_Init_u();
  delay_ms(100);
	
  while(1)
	{
		
if(rc.s2 == 3)//抓手电磁阀控制
		{
			if(rc.ch4==-660){//新抓球
				if(PEout(9) == 0){					
					PEout(9) = 1;
				delay_ms(100);}
			  else if(PEout(9) == 1){
					PEout(9) = 0;
					delay_ms(100);
			}}
			
			  if(rc.ch4==660){//新气杆
					if(PBout(14) == 0){
					PBout(14) = 1;
					delay_ms(100);}
					else if(PBout(14) == 1){
						PBout(14) = 0;
					delay_ms(100);}
				}			
		}
		
		if(rc.s1==1 && rc.s2==3){  //摩擦轮发射推球电磁阀控制
			if(rc.ch1 == 660){
				delay_ms(4600);
				if(rc.ch1 == 660){
					PBout(14) = 1;
					delay_ms(1500);
					PBout(14) = 0;
				}			
		}
			if(rc.ch3 == 660){//电动推杆控制
				PBout(12) = 1;
				PBout(13) = 0;
			}
		  if(rc.ch3 == -660){//电动推杆控制
				PBout(12) = 0;
				PBout(13) = 1;
			}
			if(rc.ch3!=660|rc.ch3!=-660){//电动推杆停
				PBout(12) = 0;
				PBout(13) = 0;
			}
	}
////		//一键取球
//		if(rc.s1==2 && rc.s2==3){
//			if(rc.ch1==660){
//				angle_zs_target = -70124.00f;
//				delay_ms(1500);
//		  	PEout(9) = 1;
//				delay_ms(800);				
//				angle_zs_target = 0.00f;
//				delay_ms(300);
//				PEout(9) = 0;
//			
//			}
			
		TIM_SetCompare1(TIM1,2000);
		delay_ms(1000);
		TIM_SetCompare1(TIM1,1000);
		delay_ms(1000);
		TIM_SetCompare1(TIM1,1500);
		delay_ms(1000);
		}
		
	}
	












