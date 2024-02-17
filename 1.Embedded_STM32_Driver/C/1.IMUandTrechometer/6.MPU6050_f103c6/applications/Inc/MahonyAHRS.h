//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MahonyAHRS_H
#define MahonyAHRS_H
#include "mpu6050_i2c.h"
#include "struct_typedef.h"
//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile fp32 twoKp;			// 2 * proportional gain (Kp)
extern volatile fp32 twoKi;			// 2 * integral gain (Ki)
extern volatile fp32 q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MahonyAHRSupdate(IMU_t *imu_data);
void MahonyAHRSupdateIMU(IMU_t *imu_data);
fp32 invSqrt(fp32 x);

void get_angle(fp32 *q,IMU_t *imu_data);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
