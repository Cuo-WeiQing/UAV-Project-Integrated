#include "Com_PID.h"

/**
 * @description: 计算pid
 * @param {PID_Struct} *pid
 * @return {*}
 */
void Com_PID_ComputePID(PID_Struct *pid)
{
    /* 0. 计算误差 */
    float error = pid->measure - pid->desire;
    /* 1. 比例项 */
    float pV = pid->kp * error;

    /* 2. 积分项 */
    pid->integral += pid->ki * error * pid->dt;

    /* 3. 微分项 */
    float dV       = pid->kd * (error - pid->lastError) / pid->dt;
    pid->lastError = error;

    /* 4. 计算结果 */
    pid->result = pV + pid->integral + dV;
}

/**
 * @description: 串级pid
 * @param {PID_Struct} *out 外环
 * @param {PID_Struct} *in 内环
 * @return {*}
 */
void Com_PID_CascadePID(PID_Struct *out, PID_Struct *in)
{
    /* 1. 计算外环 */
    Com_PID_ComputePID(out);
    /* 2. 把外环的输出作为内环的期望值 */
    in->desire = out->result;
    /* 3. 计算内环 */
    Com_PID_ComputePID(in);
}

