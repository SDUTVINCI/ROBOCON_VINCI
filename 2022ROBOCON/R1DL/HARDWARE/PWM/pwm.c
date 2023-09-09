#include "pwm.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器PWM 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/4
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 


void TIM_PWM_Init(u32 arr,u32 psc)
{		 					 
	//此部分需手动修改IO口设置
	
	
	
//	//tim1
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  	//TIM14时钟使能    
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//使能PORTF时钟	
//	
//	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); //GPIOF9复用为定时器14
//	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);
//	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1);
//	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;           //GPIOF9
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
//	GPIO_Init(GPIOE,&GPIO_InitStructure);              //初始化PF9
//	  
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
//	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//初始化定时器14
//	
//	//初始化TIM14 Channel1 PWM模式	 
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
// 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
//	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
//	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
//	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
//	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
////	TIM_OCInitStructure.TIM_Pulse = 0;
//	
//	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
//	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
//	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
//	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

//	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器
//	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
//	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
//	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
// 
//  TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPE使能 
//	
//	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
// 	TIM_CtrlPWMOutputs(TIM1,ENABLE);
		




//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM14时钟使能    
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 	//使能PORTF时钟	
//	
//	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4); //GPIOF9复用为定时器14
//	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
////	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
//	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;           //GPIOF9
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
//	GPIO_Init(GPIOD,&GPIO_InitStructure);              //初始化PF9
//	  
//	TIM_TimeBaseStructure.TIM_Prescaler=83;  //定时器分频
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 

//	
//	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器14
//	
//	//初始化TIM14 Channel1 PWM模式	 
//	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
// 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
//	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
//	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
//	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
//	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

//	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM14在CCR1上的预装载寄存器
//	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
//	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
//	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
// 
//  TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能 
//	
//	TIM_Cmd(TIM4, ENABLE);  //使能TIM14
//// 	TIM_CtrlPWMOutputs(TIM4,ENABLE);


}  
