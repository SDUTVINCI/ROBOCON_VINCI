#include "MT6701.h"
#include "chassic.h"
#include "Communition.h"

extern Rxbuffer all_rxbuff;

void MT6701_ReadAngle(I2C_HandleTypeDef *hi2c) 
{
if (hi2c->Instance == I2C2) {

    HAL_I2C_Mem_Read_DMA(hi2c, MT6701_I2C_ADDR, MT6701_I2C_DATA_ADDR_H, 
        I2C_MEMADD_SIZE_8BIT, all_rxbuff.MT6701, 2);

    robot.Launch_Angle = ((all_rxbuff.MT6701[0] << 6) | all_rxbuff.MT6701[1] >> 2) & 0x3FFF;
    robot.Launch_Angle = ((robot.Launch_Angle * 360.0f) / 16384.0f) -34.607;
    //平衡0度差值115.4
}

}
// extern "C"{
// void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
// //电机编号一
//   if (hi2c->Instance == I2C5) {
//     chassic.wheel_msgs[0].Helm_Wheel_Angle = ((buffer[0] << 6) | buffer[1 >> 2]) & 0x3FFF;
//     chassic.wheel_msgs[0].Helm_Wheel_Angle = (chassic.wheel_msgs[0].Helm_Wheel_Angle * 360.0f) / 16384.0f;
//   }

//    if (hi2c->Instance == I2C1) {
//         MT6701_ReadAngle(&hi2c1);
//     chassic.wheel_msgs[2].Helm_Wheel_Angle = ((buffer[4] << 6) | buffer[5] >> 2) & 0x3FFF;
//     chassic.wheel_msgs[2].Helm_Wheel_Angle = (chassic.wheel_msgs[2].Helm_Wheel_Angle * 360.0f) / 16384.0f;
//    }
//    if (hi2c->Instance == I2C2) {
//         MT6701_ReadAngle(&hi2c2);
//     chassic.wheel_msgs[1].Helm_Wheel_Angle = ((buffer[2] << 6) | buffer[3] >> 2) & 0x3FFF;
//     chassic.wheel_msgs[1].Helm_Wheel_Angle = (chassic.wheel_msgs[1].Helm_Wheel_Angle * 360.0f) / 16384.0f;
//    }
//  }
// }










