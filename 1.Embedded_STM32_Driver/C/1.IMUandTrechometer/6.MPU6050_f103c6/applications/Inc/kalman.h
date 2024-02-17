#ifndef __KALMAN_H_
#define __KALMAN_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "include.h"

void kalman_filter(float angle_m, float gyro_m, float *angle_f, float *angle_dot_f);


#ifdef __cplusplus
}
#endif

#endif
