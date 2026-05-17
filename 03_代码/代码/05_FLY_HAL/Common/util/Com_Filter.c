#include "Com_Filter.h"

#include "Com_FlightConfig.h"

int16_t Com_Filter_LowPass(int16_t newData, int16_t lastData)
{
    return LP_FILTER_ALPHA * lastData + (1 - LP_FILTER_ALPHA) * newData;
}


/* �������˲����� */
KalmanFilter_Struct kfs[3] = {
    {KF_INIT_Q, 0, 0, 0, KF_INIT_R, 0.543},
    {KF_INIT_Q, 0, 0, 0, KF_INIT_R, 0.543},
    {KF_INIT_Q, 0, 0, 0, KF_INIT_R, 0.543}};
double Common_Filter_KalmanFilter(KalmanFilter_Struct *kf, double input)
{
    kf->Now_P = kf->LastP + kf->Q;
    kf->Kg    = kf->Now_P / (kf->Now_P + kf->R);
    kf->out   = kf->out + kf->Kg * (input - kf->out);
    kf->LastP = (1 - kf->Kg) * kf->Now_P;
    return kf->out;
}
