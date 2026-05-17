#include "Com_PID.h"

/**
 * @description: 初始化PID结构体的安全参数
 * @param {PID_Struct} *pid
 * @param {float} integralMax 积分限幅
 * @param {float} outputMax 输出限幅
 * @param {float} deadband 微分死区
 * @return {*}
 */
void Com_PID_Init(PID_Struct *pid, float integralMax, float outputMax, float deadband)
{
    pid->integralMax = integralMax;
    pid->outputMax   = outputMax;
    pid->deadband    = deadband;
    pid->integral    = 0;
    pid->lastError   = 0;
    pid->result      = 0;
}

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

    /* 2. 积分项 (带抗积分饱和) */
    if (pid->ki != 0.0f && pid->dt > 0.0f)
    {
        pid->integral += pid->ki * error * pid->dt;

        /* 抗积分饱和: 钳位积分项 */
        if (pid->integralMax > 0.0f)
        {
            if (pid->integral > pid->integralMax)
                pid->integral = pid->integralMax;
            else if (pid->integral < -pid->integralMax)
                pid->integral = -pid->integralMax;
        }
    }

    /* 3. 微分项 (带死区保护，防止噪声放大) */
    float dV = 0.0f;
    if (pid->dt > 1e-6f) /* 防止除零 */
    {
        float errorDelta = error - pid->lastError;
        if (pid->deadband > 0.0f)
        {
            /* 微分死区: 误差变化小于阈值时微分项为零 */
            if (errorDelta > pid->deadband || errorDelta < -pid->deadband)
            {
                dV = pid->kd * errorDelta / pid->dt;
            }
        }
        else
        {
            dV = pid->kd * errorDelta / pid->dt;
        }
    }
    pid->lastError = error;

    /* 4. 输出合成 */
    pid->result = pV + pid->integral + dV;

    /* 5. 输出限幅 */
    if (pid->outputMax > 0.0f)
    {
        if (pid->result > pid->outputMax)
            pid->result = pid->outputMax;
        else if (pid->result < -pid->outputMax)
            pid->result = -pid->outputMax;
    }
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
    /* 2. 外环的输出作为内环的期望值 */
    in->desire = out->result;
    /* 3. 计算内环 */
    Com_PID_ComputePID(in);
}
