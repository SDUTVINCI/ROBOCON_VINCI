//=====================================================================================================
// MahonyAHRS.c
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

//---------------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include <math.h>
//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	512.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile fp32 twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile fp32 twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile fp32 q[4] = {1.0f , 0.0f , 0.0f , 0.0f};					// quaternion of sensor frame relative to auxiliary frame
volatile fp32 integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRSupdate(IMU_t *imu_data) 
{
	float recipNorm;
	fp32 qq[4][4];
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((imu_data->Magnet.X == 0.0f) && (imu_data->Magnet.Y == 0.0f) && (imu_data->Magnet.Z == 0.0f)) 
	{
		MahonyAHRSupdateIMU(imu_data);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((imu_data->Accel.X == 0.0f) && (imu_data->Accel.Y == 0.0f) && (imu_data->Accel.Z == 0.0f))) 
	{

		// Normalise accelerometer measurement
		recipNorm = invSqrt(imu_data->Accel.X * imu_data->Accel.X + imu_data->Accel.Y * imu_data->Accel.Y + imu_data->Accel.Z * imu_data->Accel.Z);
		imu_data->Accel.X *= recipNorm;
		imu_data->Accel.Y *= recipNorm;
		imu_data->Accel.Z *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(imu_data->Magnet.X * imu_data->Magnet.X + imu_data->Magnet.Y * imu_data->Magnet.Y + imu_data->Magnet.Z * imu_data->Magnet.Z);
		imu_data->Magnet.X *= recipNorm;
		imu_data->Magnet.Y *= recipNorm;
		imu_data->Magnet.Z *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        qq[0][0] = q[0] * q[0];
        qq[0][1] = q[0] * q[1];
        qq[0][2] = q[0] * q[2];
        qq[0][3] = q[0] * q[3];
        qq[1][1] = q[1] * q[1];
        qq[1][2] = q[1] * q[2];
        qq[1][3] = q[1] * q[3];
        qq[2][2] = q[2] * q[2];
        qq[2][3] = q[2] * q[3];
        qq[3][3] = q[3] * q[3];   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (imu_data->Magnet.X * (0.5f - qq[2][2] - qq[3][3]) + imu_data->Magnet.Y * (qq[1][2] - qq[0][3]) + imu_data->Magnet.Z * (qq[1][3] + qq[0][2]));
        hy = 2.0f * (imu_data->Magnet.X * (qq[1][2] + qq[0][3]) + imu_data->Magnet.Y * (0.5f - qq[1][1] - qq[3][3]) + imu_data->Magnet.Z * (qq[2][3] - qq[0][1]));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (imu_data->Magnet.X * (qq[1][3] - qq[0][2]) + imu_data->Magnet.Y * (qq[2][3] + qq[0][1]) + imu_data->Magnet.Z * (0.5f - qq[1][1] - qq[2][2]));

		// Estimated direction of gravity and magnetic field
		halfvx = qq[1][3] - qq[0][2];
		halfvy = qq[0][1] + qq[2][3];
		halfvz = qq[0][0] - 0.5f + qq[3][3];
        halfwx = bx * (0.5f - qq[2][2] - qq[3][3]) + bz * (qq[1][3] - qq[0][2]);
        halfwy = bx * (qq[1][2] - qq[0][3]) + bz * (qq[0][1] + qq[2][3]);
        halfwz = bx * (qq[0][2] + qq[1][3]) + bz * (0.5f - qq[1][1] - qq[2][2]);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (imu_data->Accel.Y * halfvz - imu_data->Accel.Z * halfvy) + (imu_data->Magnet.Y * halfwz - imu_data->Magnet.Z * halfwy);
		halfey = (imu_data->Accel.Z * halfvx - imu_data->Accel.X * halfvz) + (imu_data->Magnet.Z * halfwx - imu_data->Magnet.X * halfwz);
		halfez = (imu_data->Accel.X * halfvy - imu_data->Accel.Y * halfvx) + (imu_data->Magnet.X * halfwy - imu_data->Magnet.Y * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			imu_data->Gyro.X += integralFBx;	// apply integral feedback
			imu_data->Gyro.Y += integralFBy;
			imu_data->Gyro.Z += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		imu_data->Gyro.X += twoKp * halfex;
		imu_data->Gyro.Y += twoKp * halfey;
		imu_data->Gyro.Z += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	imu_data->Gyro.X *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	imu_data->Gyro.Y *= (0.5f * (1.0f / sampleFreq));
	imu_data->Gyro.Z *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * imu_data->Gyro.X - qc * imu_data->Gyro.Y - q[3] * imu_data->Gyro.Z);
	q[1] += ( qa * imu_data->Gyro.X + qc * imu_data->Gyro.Z - q[3] * imu_data->Gyro.Y);
	q[2] += ( qa * imu_data->Gyro.Y - qb * imu_data->Gyro.Z + q[3] * imu_data->Gyro.X);
	q[3] += ( qa * imu_data->Gyro.Z + qb * imu_data->Gyro.Y - qc * imu_data->Gyro.X); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(IMU_t *imu_data) 
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((imu_data->Accel.X == 0.0f) && (imu_data->Accel.Y == 0.0f) && (imu_data->Accel.Z == 0.0f))) 
		{

		// Normalise accelerometer measurement
		recipNorm = invSqrt(imu_data->Accel.X * imu_data->Accel.X + imu_data->Accel.Y * imu_data->Accel.Y + imu_data->Accel.Z * imu_data->Accel.Z);
		imu_data->Accel.X *= recipNorm;
		imu_data->Accel.Y *= recipNorm;
		imu_data->Accel.Z *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (imu_data->Accel.Y * halfvz - imu_data->Accel.Z * halfvy);
		halfey = (imu_data->Accel.Z * halfvx - imu_data->Accel.X * halfvz);
		halfez = (imu_data->Accel.X * halfvy - imu_data->Accel.Y * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			imu_data->Gyro.X = integralFBx;	// apply integral feedback
			imu_data->Gyro.Y += integralFBy;
			imu_data->Gyro.Z += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		imu_data->Gyro.X += twoKp * halfex;
		imu_data->Gyro.Y += twoKp * halfey;
		imu_data->Gyro.Z += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	imu_data->Gyro.X *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	imu_data->Gyro.Y *= (0.5f * (1.0f / sampleFreq));
	imu_data->Gyro.Z *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * imu_data->Gyro.X - qc * imu_data->Gyro.Y - q[3] * imu_data->Gyro.Z);
	q[1] += (qa * imu_data->Gyro.X + qc * imu_data->Gyro.Z - q[3] * imu_data->Gyro.Y);
	q[2] += (qa * imu_data->Gyro.Y - qb * imu_data->Gyro.Z + q[3] * imu_data->Gyro.X);
	q[3] += (qa * imu_data->Gyro.Z + qb * imu_data->Gyro.Y - qc * imu_data->Gyro.X); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}


//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


void get_angle(fp32 *q, IMU_t *imu_data) 
	{ 
		imu_data->Euler.yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f);
		imu_data->Euler.pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2])); 
		imu_data->Euler.roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f); 
	}




//====================================================================================================
// END OF CODE
//====================================================================================================
