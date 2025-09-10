#include "chassic.h"
#include "DR16.h"
#include "fdcan_receive.h"
#include "pid.h"
#include "pid_user.h"
#include <math.h>
#include "cmsis_os.h"
#include "Communition.h"
#include "my_task.h"
/************************** 

车轮顺序（CANid）：    
          1（1）      2（2）
                3（3）        
*/

extern Rxbuffer all_rxbuff;
ROBOT robot;


void ROBOT::CHASSIC::Omni_Move_Calc()
{
//右手坐标三轮全向
this->Motor_Speed_Target[0]=-Chassic_Move_Data.vy *sin(PI/6.0f)+Chassic_Move_Data.vx *cos(PI/6.0f) -Chassic_Move_Data.vomega;
this->Motor_Speed_Target[1]=-Chassic_Move_Data.vy *sin(PI/6.0f)-Chassic_Move_Data.vx *cos(PI/6.0f) -Chassic_Move_Data.vomega;
this->Motor_Speed_Target[2]= Chassic_Move_Data.vy-0-Chassic_Move_Data.vomega;

}

void ROBOT::CHASSIC::Chassic_Move()
{
    fp32 v_3508[4]={0};

    v_3508[0]=pid_controller.can_motor.CAN1_Velocity_Realize(Motor_Speed_Target[0], 1);
    v_3508[1]=pid_controller.can_motor.CAN1_Velocity_Realize(Motor_Speed_Target[1], 2);
    v_3508[2]=pid_controller.can_motor.CAN1_Velocity_Realize(Motor_Speed_Target[2], 3);

    can_bus.cmd.FDCAN1_Front(v_3508[0], v_3508[1], v_3508[2], 0);
		
}
void ROBOT::CHASSIC::Chassic_Mode()
{
  //手柄
    if (all_rxbuff.s2 == MANUAL_MODE)
    {
       this->Chassic_Move_Data.vx=5.0*(all_rxbuff.remote_sense[3]-1024);
       this->Chassic_Move_Data.vy=-5.0*(all_rxbuff.remote_sense[2]-1024);
       this->Chassic_Move_Data.vomega=-2.5 * (all_rxbuff.remote_sense[0]-1024);
    }
    //上位机
    if (all_rxbuff.s2 == AUTO_MODE)
    {
		// robot.chassic.Chassic_Move_Data.vx = 
        //     robot.upper_data[0]*(60/(2*PI*WHEEL_Radius))*3591.0/187.0;
        // robot.chassic.Chassic_Move_Data.vy = 
        //     robot.upper_data[1]*(60/(2*PI*WHEEL_Radius))*3591.0/187.0;
        // robot.chassic.Chassic_Move_Data.vomega = 
        //     robot.upper_data[2]*Radius*(60/(2*PI*WHEEL_Radius)*3591.0/187.0);
    }

}

/**
 * @brief  弧度转换为角度
 * @param  rad 弧度值
 * @retval deg 角度值
 * @attention
 */
fp32 ROBOT::CHASSIC::rad2deg(fp32 rad)
{
	fp32 deg;
	deg = (fp32)(rad * (180.0f / PI));
	return deg;
}