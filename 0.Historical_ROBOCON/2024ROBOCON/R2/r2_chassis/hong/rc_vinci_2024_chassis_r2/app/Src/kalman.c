#include "kalman.h"

void InitKalmanFilter(KALMAN_FILTER *pKalmanFilter)
{
                pKalmanFilter->xk_1 = 0;
                pKalmanFilter->Pk_1 = 1;
                pKalmanFilter->xkk_1 = 0;
                pKalmanFilter->Pkk_1 = 0;
                pKalmanFilter->Kk = 0;
                pKalmanFilter->xk = 0;
                pKalmanFilter->Pk = 0;
                pKalmanFilter->Q = 0;
                pKalmanFilter->R = 0.1f;
}

float CalcKalmanFilter(KALMAN_FILTER *pKalmanFilter, float zk)
{
                // prediction
                pKalmanFilter->xkk_1 = pKalmanFilter->xk_1;
                pKalmanFilter->Pkk_1 = pKalmanFilter->Pk_1 + pKalmanFilter->Q;

                // correction
                pKalmanFilter->Kk = pKalmanFilter->Pkk_1 / (pKalmanFilter->Pkk_1 + pKalmanFilter->R);
                pKalmanFilter->xk = pKalmanFilter->xkk_1 + pKalmanFilter->Kk * (zk - pKalmanFilter->xkk_1);
                pKalmanFilter->Pk = (1.0f - pKalmanFilter->Kk) * pKalmanFilter->Pkk_1;

                pKalmanFilter->xk_1 = pKalmanFilter->xk;
                pKalmanFilter->Pk_1 = pKalmanFilter->Pk;

                return pKalmanFilter->xk;
}
