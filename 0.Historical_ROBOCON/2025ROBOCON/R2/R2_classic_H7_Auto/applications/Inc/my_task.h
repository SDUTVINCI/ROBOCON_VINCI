#ifndef __MY_TASK_H_
#define __MY_TASK_H_


#ifdef __cplusplus
extern "C"
{
#endif 


#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "usart.h"
#include "fdcan.h"
#include "gpio.h"
#include "tim.h"



//3 自瞄 
//4 上位机返回自瞄任务
//5 固定定点投篮

typedef enum 
{
    ASK_START_POS = 0X01,                   //1 请求返回原点
    ALLOW_START_POS,                        //2 上位机返回的 vx,vy,vomega
    ASK_LAUNCH_SPEED,                       //3请求投篮(自动瞄准)
    ALLOW_LAUNCH,                           //4上位机返回自瞄投篮发射参数（Vomega，speed_rpm）
    ASK_FIXED_POS_LAUNCH,                   //5预选赛固定点位投篮
    ALLOW_FIXED_POS_LAUNCH,                 //6上位机返回固定点位位置(Vx，Vy,Vomega,speed_rpm)
    UPPER_OK = 0xff                         //上位机收到请求(可以不用)
}CMD_PACKAGE;

//选择机器人模式 通过右边拨杆
#define MANUAL_MODE  3//手动模式
#define AUTO_MODE    1//自动模式


#define ROD_BOTTOM_PUSH  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET)        //0的时候是缩起来
#define ROD_BOTTOM_BACK  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET)
#define ROD_TOP_PUSH     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET)
#define ROD_TOP_BACK     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET)

#define BALL_SWITCH      HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)   
//有球在是0 没有球在是1
#define BALL_IN 0
#define BALL_OUT 1



void Start_Dribble();
void Catch_the_Ball();
bool Judge_ball(void);
void UpdateSpeedFromBluetooth(int16_t new_speed);
void DT7_Switch_Update();

void StartLaunchTask(void *argument);
void BT_Auto_Fire_Task(void *argument);

#ifdef __cplusplus
}
#endif 
#endif