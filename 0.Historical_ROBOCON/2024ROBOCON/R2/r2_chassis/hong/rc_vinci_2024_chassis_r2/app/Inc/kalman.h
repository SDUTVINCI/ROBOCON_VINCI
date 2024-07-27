#ifndef _kal_
#define _kal_

#include "include.h"
typedef struct _KALMAN_FILTER
{
                float xk_1;  // x(k-1|k-1)
                float Pk_1;  // P(k-1|k-1)
                float xkk_1; // x(k|k-1)
                float Pkk_1; // P(k|k-1)
                float Kk;    // Kk
                float xk;    // x(k|k)
                float Pk;    // P(k|k)
                float Q;
                float R;
} KALMAN_FILTER;

void InitKalmanFilter(KALMAN_FILTER *pKalmanFilter);
float CalcKalmanFilter(KALMAN_FILTER *pKalmanFilter, float zk);
#endif
