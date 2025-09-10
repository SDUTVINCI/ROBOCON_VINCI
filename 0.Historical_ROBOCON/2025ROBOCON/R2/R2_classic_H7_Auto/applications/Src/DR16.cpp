#include "DR16.h"
#include "chassic.h"
#include "Communition.h"

extern Rxbuffer all_rxbuff;

void Get_Remote_Date()
{
    all_rxbuff.remote_sense[0]= ((int16_t)all_rxbuff.DR16[0]| ((int16_t)all_rxbuff.DR16[1] << 8)) & 0x07ff;   
    //!< Channel 0
    all_rxbuff.remote_sense[1] =(((int16_t)all_rxbuff.DR16[1] >> 3) | ((int16_t)all_rxbuff.DR16[2] << 5)) & 0x07ff; 
    //!< Channel 1 
    all_rxbuff.remote_sense[2] = (((int16_t)all_rxbuff.DR16[2] >> 6) | ((int16_t)all_rxbuff.DR16[3] << 2) |((int16_t)all_rxbuff.DR16[4] << 10))& 0x07ff; 
    //!< Channel 2
    all_rxbuff.remote_sense[3] =(((int16_t)all_rxbuff.DR16[4] >> 1) | ((int16_t)all_rxbuff.DR16[5] << 7)) & 0x07ff;
    //!< Channel 3 

    //satori:???????????5???
    /* prevent remote control zero deviation */
    // if(all_rxbuff.remote_sense[0] <= 9 && all_rxbuff.remote_sense[0] >= -9)
    //     all_rxbuff.remote_sense[0] = 0;
    if(all_rxbuff.remote_sense[1] <= 5 && all_rxbuff.remote_sense[1] >= -5)
        all_rxbuff.remote_sense[1] = 0;
    if(all_rxbuff.remote_sense[2] <= 5 && all_rxbuff.remote_sense[2] >= -5)
        all_rxbuff.remote_sense[2] = 0;
    if(all_rxbuff.remote_sense[3] <= 5 && all_rxbuff.remote_sense[3] >= -5)
        all_rxbuff.remote_sense[3] = 0;
    all_rxbuff.s1 = ((all_rxbuff.DR16[5] >> 4) & 0x000C) >> 2;
    all_rxbuff.s2 = ((all_rxbuff.DR16[5] >> 4) & 0x0003);

    robot.target_ball_angle = (-1) * (all_rxbuff.remote_sense[1] - 1024)/10.0f;//550/10 = 55?
    
    HAL_UART_Receive_DMA(&huart5, all_rxbuff.DR16, 18);
}



/*Ò£¸ÐÊÔÍ¼
    S1                S2
    ^3               ^1
    |                |
----|--->2       ----|---->0
    |                |   
 */