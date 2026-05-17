#include "Com_IMU.h"
#include "Com_FlightConfig.h"
#include "math.h"

/* ============================欧拉角计算================================== */
/* ===============================开始===================================== */

/* 计算欧拉角用到的3个常量 */
float RtA = 57.2957795f;   // 弧度->度
// 陀螺仪初始量程+-2000度/秒 1/(65536 / 4000) = 0.03051756*2
float Gyro_G = 4000.0 / 65536;   // 度/s
// 度每秒,转换成弧度每秒
float Gyro_Gr = 4000.0 / 65536 / 180 * 3.1415926;   // 弧度/s

#define squa(Sq) (((float)Sq) * ((float)Sq))        /* 求平方 */

/**
 * @description: 快速计算 1/sqrt(num)
 * @param {float} number
 */
static float Q_rsqrt(float number)
{
    long        i;
    float       x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y  = number;
    i  = *(long *)&y;
    i  = 0x5f3759df - (i >> 1);
    y  = *(float *)&i;
    y  = y * (threehalfs - (x2 * y * y));   // 1st iteration 一次牛顿迭代
    return y;
}

static double normAccz; /* z轴上的加速度 */

/**
 * @description: 根据mpu的6个数据, 获取飞行器当前的欧拉角
 * @param {GyroAccel_Struct} *gyroAccel mpu的6个数据
 * @param {EulerAngle_Struct} *EulerAngle 计算得到的欧拉角
 * @param {float} dt 运行周期 (单位s)
 * @return {*}
 */
void Common_IMU_GetEulerAngle(GyroAccel_Struct  *gyroAccel,
                              EulerAngle_Struct *eulerAngle,
                              float              dt)
{
    volatile struct V
    {
        float x;
        float y;
        float z;
    } Gravity, Acc, Gyro, AccGravity;

    static struct V          GyroIntegError = {0};
    static Quaternion_Struct NumQ           = {1, 0, 0, 0};
    float                    q0_t, q1_t, q2_t, q3_t;
    float                    NormQuat;
    float                    HalfTime = dt * 0.5f;

    // 获取有效旋转矩阵中的重力分量
    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);
    // 加速度归一化
    NormQuat = Q_rsqrt(squa(gyroAccel->accel.accelX) +
                       squa(gyroAccel->accel.accelY) +
                       squa(gyroAccel->accel.accelZ));

    Acc.x = gyroAccel->accel.accelX * NormQuat;
    Acc.y = gyroAccel->accel.accelY * NormQuat;
    Acc.z = gyroAccel->accel.accelZ * NormQuat;
    // 向量叉积得出误差值
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
    // 加速度积分补偿角速度的漂移值 (带积分限幅防饱和)
    GyroIntegError.x += AccGravity.x * IMU_KI_DEF;
    GyroIntegError.y += AccGravity.y * IMU_KI_DEF;
    GyroIntegError.z += AccGravity.z * IMU_KI_DEF;

    /* 积分误差限幅 (Anti-windup for complementary filter) */
    if (GyroIntegError.x > IMU_KI_LIMIT)  GyroIntegError.x = IMU_KI_LIMIT;
    if (GyroIntegError.x < -IMU_KI_LIMIT) GyroIntegError.x = -IMU_KI_LIMIT;
    if (GyroIntegError.y > IMU_KI_LIMIT)  GyroIntegError.y = IMU_KI_LIMIT;
    if (GyroIntegError.y < -IMU_KI_LIMIT) GyroIntegError.y = -IMU_KI_LIMIT;
    if (GyroIntegError.z > IMU_KI_LIMIT)  GyroIntegError.z = IMU_KI_LIMIT;
    if (GyroIntegError.z < -IMU_KI_LIMIT) GyroIntegError.z = -IMU_KI_LIMIT;

    // 角速度融合加速度积分补偿值
    Gyro.x = gyroAccel->gyro.gyroX * Gyro_Gr + IMU_KP_DEF * AccGravity.x + GyroIntegError.x;   // 弧度制
    Gyro.y = gyroAccel->gyro.gyroY * Gyro_Gr + IMU_KP_DEF * AccGravity.y + GyroIntegError.y;
    Gyro.z = gyroAccel->gyro.gyroZ * Gyro_Gr + IMU_KP_DEF * AccGravity.z + GyroIntegError.z;

    // 一阶龙格库塔, 更新四元数
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;

    // 四元数归一化
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    /*机体坐标系下的Z方向量*/
    float vecxZ = 2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3;     /*矩阵(3,1)项*/
    float vecyZ = 2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;     /*矩阵(3,2)项*/
    float veczZ = 1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2; /*矩阵(3,3)项*/

    float yaw_G = gyroAccel->gyro.gyroZ * Gyro_G;   // 把Z角速度原始值 转换为Z角度/秒      Gyro_G陀螺仪初始量程+-2000度每秒 1 / (65536 / 4000) = 0.03051756*2
    if((yaw_G > 0.5f) || (yaw_G < -0.5))            // 运动太小认为是噪声，避免偏航漂移
    {
        eulerAngle->yaw += yaw_G * dt;   // 角速度积分出偏航角
    }

    eulerAngle->pitch = asin(vecxZ) * RtA;   // 俯仰角

    eulerAngle->roll = atan2f(vecyZ, veczZ) * RtA;   // 横滚角

    normAccz = gyroAccel->accel.accelX * vecxZ + gyroAccel->accel.accelY * vecyZ + gyroAccel->accel.accelZ * veczZ; /*Z轴垂直方向上的加速度（注意:机身倾斜时，Z轴加速度的测量值会降低，但是垂直方向上的加速度应该保持常值）*/
}

/**
 * @description: 获取Z轴上的加速度 (如果机身倾斜,会考虑z轴上加速度的合成)
 * @return {*}
 */
float Common_IMU_GetNormAccZ(void)
{
    return normAccz;
}
/* ======================欧拉角计算================================== */
/* ========================结束==================================== */
