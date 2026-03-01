#ifndef __COM_CONFIG_H
#define __COM_CONFIG_H

#include "stdint.h"
#include "stdio.h"

typedef enum
{
    Com_OK = 0,
    Com_TIMEOUT,
    Com_FAIL
} Com_Status;

typedef enum
{
    LEFT_TOP,
    LEFT_BOTTOM,
    RIGHT_TOP,
    RIGHT_BOOTTOM
} Com_Location;

typedef struct
{
    /* 表示灯的位置 */
    Com_Location location;

    /* 表示灯的状态:
        0: 常亮
        1: 常灭
        2+: 闪烁周期
     */
    uint8_t status;
} LedStruct;

/* 表示电机的结构体 */
typedef struct
{
    Com_Location location;
    int16_t      speed;
} Motor_Struct;

typedef struct
{
    int16_t THR; /* 油门 */
    int16_t PIT; /* 俯仰 */
    int16_t ROL; /* 横滚 */
    int16_t YAW; /* 偏航 */

    uint8_t isPowerDown; /* 是否关机: 1:关机 0:不关机 */
    uint8_t isFixHeight; /* 是否翻转定高的状态 */
} JoyStick_Struct;

/* 定义油门解锁时的几个状态 */
typedef enum
{
    THR_FREE = 0,
    THR_MAX,
    THR_MAX_LEAVE,
    THR_MIN,
    THR_UNLOCK
} Com_RemoteStatus;

/* 存储角速度 */
typedef struct
{
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
} Gyro_Struct;

/* 存储加速度 */
typedef struct
{
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
} Accel_Struct;

/* 角速度和加速度 */
typedef struct
{
    Gyro_Struct  gyro;
    Accel_Struct accel;
} GyroAccel_Struct;

/* 欧拉角 */
typedef struct
{
    float pitch;
    float roll;
    float yaw;

} EulerAngle_Struct;

/* 表示四元数的结构体 */
typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_Struct;

/* 表示pid 结构体 */
typedef struct
{
    float kp; /* 比例系数 */
    float ki; /* 积分系数 */
    float kd; /* 微分系数 */

    float dt; /* 采样时间 */

    float integral;  /*保存 积分值 */
    float lastError; /* 上次误差 */

    float desire; /* 期望值 */
    float measure; /* 测量值 */

    float result;  /* pid最终结果 */ 

} PID_Struct;

/* 2.4g通讯的数据负载的自定义帧头 */
#define FRAME_0 (0x11)
#define FRAME_1 (0x22)
#define FRAME_2 (0x33)

#define LIMIT(x, min, max) (x) >= (max) ? (max) : ((x) <= (min) ? (min) : (x))

extern LedStruct ledLeftTop;
extern LedStruct ledLeftBootom;
extern LedStruct ledRightTop;
extern LedStruct ledRightBottom;

extern Com_Status isRemoteConnected;
extern Com_Status isRemoteUnlocked;
extern Com_Status isFixHeight;

extern Motor_Struct motorLeftTop;
extern Motor_Struct motorLeftBottom;
extern Motor_Struct motorRightTop;
extern Motor_Struct motorRightBottom;

extern JoyStick_Struct joyStick;

extern GyroAccel_Struct gyroAccel;

extern EulerAngle_Struct eulerAngle;

void Com_Config_PrintJoyStick(uint8_t *pre);

void Com_Config_PrintGyroAccel(uint8_t *pre, GyroAccel_Struct *gyroAccel);

void Com_Config_PrintEulerAngle(uint8_t *pre, EulerAngle_Struct *eulerAngle);

#endif
