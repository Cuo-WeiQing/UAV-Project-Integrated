#include "App_Flight.h"
#include "Com_FlightConfig.h"

/* ====================== 姿态/速率PID ====================== */
/* 俯仰 */
PID_Struct pitchPID = {.kp = PITCH_OUT_KP, .ki = PITCH_OUT_KI, .kd = PITCH_OUT_KD};
PID_Struct gyroYPID = {.kp = PITCH_IN_KP,  .ki = PITCH_IN_KI,  .kd = PITCH_IN_KD};

/* 横滚 */
PID_Struct rollPID  = {.kp = ROLL_OUT_KP,  .ki = ROLL_OUT_KI,  .kd = ROLL_OUT_KD};
PID_Struct gyroXPID = {.kp = ROLL_IN_KP,   .ki = ROLL_IN_KI,   .kd = ROLL_IN_KD};

/* 偏航 */
PID_Struct yawPID   = {.kp = YAW_OUT_KP,   .ki = YAW_OUT_KI,   .kd = YAW_OUT_KD};
PID_Struct gyroZPID = {.kp = YAW_IN_KP,    .ki = YAW_IN_KI,    .kd = YAW_IN_KD};

/* 定高 */
PID_Struct heightPID = {.kp = HEIGHT_OUT_KP, .ki = HEIGHT_OUT_KI, .kd = HEIGHT_OUT_KD};
PID_Struct zSpeedPID = {.kp = ZSPEED_IN_KP,  .ki = ZSPEED_IN_KI,  .kd = ZSPEED_IN_KD};

/**
 * @description: 初始化PID安全参数
 */
static void App_Flight_PIDInit(void)
{
    /* 姿态外环 */
    Com_PID_Init(&pitchPID, POSTURE_OUT_INTEGRAL_MAX, POSTURE_OUT_OUTPUT_MAX, POSTURE_OUT_DEADBAND);
    Com_PID_Init(&rollPID,  POSTURE_OUT_INTEGRAL_MAX, POSTURE_OUT_OUTPUT_MAX, POSTURE_OUT_DEADBAND);
    Com_PID_Init(&yawPID,   POSTURE_OUT_INTEGRAL_MAX, POSTURE_OUT_OUTPUT_MAX, POSTURE_OUT_DEADBAND);

    /* 速率内环 */
    Com_PID_Init(&gyroYPID, GYRO_IN_INTEGRAL_MAX, GYRO_IN_OUTPUT_MAX, GYRO_IN_DEADBAND);
    Com_PID_Init(&gyroXPID, GYRO_IN_INTEGRAL_MAX, GYRO_IN_OUTPUT_MAX, GYRO_IN_DEADBAND);
    Com_PID_Init(&gyroZPID, GYRO_IN_INTEGRAL_MAX, GYRO_IN_OUTPUT_MAX, GYRO_IN_DEADBAND);

    /* 定高外环 */
    Com_PID_Init(&heightPID, HEIGHT_OUT_INTEGRAL_MAX, HEIGHT_OUT_OUTPUT_MAX, HEIGHT_OUT_DEADBAND);

    /* Z速率内环 */
    Com_PID_Init(&zSpeedPID, ZSPEED_IN_INTEGRAL_MAX, ZSPEED_IN_OUTPUT_MAX, ZSPEED_IN_DEADBAND);
}

/**
 * @description: 飞行模块启动
 * @return {*}
 */
void App_Flight_Start(void)
{
    /* 0. 初始化PID安全参数 */
    App_Flight_PIDInit();

    /* 1. 初始化motor */
    debug_printfln("初始化电机 开始");
    Inf_Motor_Init();
    debug_printfln("初始化电机 结束");

    /* 2. 初始化MPU6050 */
    debug_printfln("初始化MPU6050 开始");
    Inf_MPU6050_Init();
    debug_printfln("初始化MPU6050 结束");

    /* 3. 初始化VL53L1X */
    debug_printfln("初始化激光传感 开始");
    Inf_VL53LX1_Init();
    debug_printfln("初始化激光传感 结束");
}

/**
 * @description: 飞控姿态数据滤波
 *  角速度: 使用一阶低通滤波
 *      影响小, 噪声影响小
 *  加速度: 卡尔曼滤波
 *      噪声大, 容易受振动影响
 * @param {GyroAccel_Struct} *gyroAccel
 * @return {*}
 */
void App_Flight_GetGyroAccelWithFilter(GyroAccel_Struct *gyroAccel)
{
    /* 1. 读取原始数据 */
    taskENTER_CRITICAL();
    Inf_MPU6050_ReadGyroAccelCalibrated(gyroAccel);
    taskEXIT_CRITICAL();

    /* 2. 对角速度做一阶低通通滤波 */
    static int16_t lastDatas[3] = {0};
    gyroAccel->gyro.gyroX       = Com_Filter_LowPass(gyroAccel->gyro.gyroX, lastDatas[0]);
    gyroAccel->gyro.gyroY       = Com_Filter_LowPass(gyroAccel->gyro.gyroY, lastDatas[1]);
    gyroAccel->gyro.gyroZ       = Com_Filter_LowPass(gyroAccel->gyro.gyroZ, lastDatas[2]);
    lastDatas[0]                = gyroAccel->gyro.gyroX;
    lastDatas[1]                = gyroAccel->gyro.gyroY;
    lastDatas[2]                = gyroAccel->gyro.gyroZ;

    /* 3. 对加速度做卡尔曼滤波 */
    gyroAccel->accel.accelX = Common_Filter_KalmanFilter(&kfs[0], gyroAccel->accel.accelX);
    gyroAccel->accel.accelY = Common_Filter_KalmanFilter(&kfs[1], gyroAccel->accel.accelY);
    gyroAccel->accel.accelZ = Common_Filter_KalmanFilter(&kfs[2], gyroAccel->accel.accelZ);
}

/**
 * @description: 获取欧拉角
 * @return {*}
 */
void App_Flight_GetEulerAngle(GyroAccel_Struct  *gyroAccel,
                              EulerAngle_Struct *eulerAngle,
                              float              dt)
{
    Common_IMU_GetEulerAngle(gyroAccel, eulerAngle, dt);
}

/**
 * @description: 计算姿态pid
 * @param {GyroAccel_Struct} *gyroAccel
 * @param {EulerAngle_Struct} *eulerAngle
 * @param {float} dt 运行周期
 * @return {*}
 */
void App_Flight_PIDPosture(GyroAccel_Struct *gyroAccel, EulerAngle_Struct *eulerAngle, float dt)
{
    /* 俯仰 */
    pitchPID.dt      = dt;
    pitchPID.desire  = (joyStick.PIT - JOYSTICK_MIDPOINT) * JOYSTICK_ANGLE_SCALE; /* 控制方向 */
    pitchPID.measure = eulerAngle->pitch;

    gyroYPID.dt      = dt;
    gyroYPID.measure = gyroAccel->gyro.gyroY * Gyro_G;

    Com_PID_CascadePID(&pitchPID, &gyroYPID);

    /* 横滚 */
    rollPID.dt      = dt;
    rollPID.desire  = (joyStick.ROL - JOYSTICK_MIDPOINT) * JOYSTICK_ANGLE_SCALE;
    rollPID.measure = eulerAngle->roll;

    gyroXPID.dt      = dt;
    gyroXPID.measure = gyroAccel->gyro.gyroX * Gyro_G;

    Com_PID_CascadePID(&rollPID, &gyroXPID);

    /* 偏航 */
    yawPID.dt      = dt;
    yawPID.desire  = (joyStick.YAW - JOYSTICK_MIDPOINT) * JOYSTICK_ANGLE_SCALE;
    yawPID.measure = eulerAngle->yaw;

    gyroZPID.dt      = dt;
    gyroZPID.measure = gyroAccel->gyro.gyroZ * Gyro_G;

    Com_PID_CascadePID(&yawPID, &gyroZPID);
}

/**
 * @description: 姿态PID赋值到电机
 * @param {Com_Status} isRemoteUnlock
 * @return {*}
 */
void App_Flight_MotorWithPosturePID(Com_Status isRemoteUnlock)
{
    if(isRemoteUnlock != Com_OK) return;

    /*
        leftTop leftBottom rightTop rightBottom

        横滚:  gyroXPID.result
            leftTop + leftBottom  vs   rightTop + rightBottom

        俯仰: gyroYPID.result
            leftTop + rightTop    vs    leftBottom + rightBottom

        偏航: gyroZPID.result
            leftTop+rightBottom   vs    leftBottom + rightTop

    */
    int16_t speed          = joyStick.THR * MOTOR_THR_SCALE;
    motorLeftTop.speed     = speed + gyroXPID.result + gyroYPID.result + gyroZPID.result;
    motorLeftBottom.speed  = speed + gyroXPID.result - gyroYPID.result - gyroZPID.result;
    motorRightTop.speed    = speed - gyroXPID.result + gyroYPID.result - gyroZPID.result;
    motorRightBottom.speed = speed - gyroXPID.result - gyroYPID.result + gyroZPID.result;

    /* 电机输出限幅 */
    motorLeftTop.speed     = LIMIT(motorLeftTop.speed,     0, MOTOR_OUTPUT_LIMIT);
    motorLeftBottom.speed  = LIMIT(motorLeftBottom.speed,  0, MOTOR_OUTPUT_LIMIT);
    motorRightTop.speed    = LIMIT(motorRightTop.speed,    0, MOTOR_OUTPUT_LIMIT);
    motorRightBottom.speed = LIMIT(motorRightBottom.speed, 0, MOTOR_OUTPUT_LIMIT);
}

/**
 * @description: 获取飞机当前的飞行高度
 * @return {*} 高度: mm
 */
uint16_t App_Flight_GetHeight(void)
{
    static uint16_t lastHeight = 0;
    uint16_t        height     = Inf_VL53LX1_GetHeight();

    if(abs(height - lastHeight) > HEIGHT_JUMP_THRESHOLD || /* 如果跳变,则返回上次的值 */
       abs(joyStick.PIT - JOYSTICK_MIDPOINT) > HEIGHT_HORIZONTAL_THRESHOLD ||  /* 非水平方向, 返回上次的值 */
       abs(joyStick.ROL - JOYSTICK_MIDPOINT) > HEIGHT_HORIZONTAL_THRESHOLD)
    {
        return lastHeight;
    }

    height     = Com_Filter_LowPass(height, lastHeight);
    lastHeight = height;

    return height;
}

/**
 * @description: 高度pid计算
 * @param {Com_Status} isRemoteUnlocked
 * @param {uint16_t} height
 * @return {*}
 */
void App_Flight_PIDHeight(Com_Status isRemoteUnlocked, uint16_t height, float dt)
{
    /* 状态机:
        状态0: 检测是否定高
        状态1: 记录油门基础值和当前高度值  固化油门: 固定的高度
        状态2: 正式pid计算
     */
    static uint8_t  status     = 0;
    static uint16_t thrHold    = 0;
    static uint16_t heightHold = 0;

    static float staticAcc = 0;  /* 静态时z的加速度 */
    if(isRemoteUnlocked == Com_OK && staticAcc == 0)
    {
        staticAcc = Common_IMU_GetNormAccZ();
    }

    switch(status)
    {
        case 0: /* 定高检测 */
        {
            /* pid归零 */
            heightPID.result = 0;
            zSpeedPID.result = 0;
            if(isRemoteUnlocked == Com_OK && isFixHeight == Com_OK)
            {
                status = 1;
            }

            break;
        }

        case 1: /* pid开始前准备 */
        {
            thrHold    = joyStick.THR;
            heightHold = height;
            status     = 2;
            break;
        }
        case 2: /* pid计算 */
        {
            /* 退出条件: 油门变化超过100,或定高标志置为0。 则取消定高 */
            if(abs(joyStick.THR - thrHold) > 100 || isFixHeight == Com_FAIL)
            {
                status               = 0; /* 回到状态0 */
                joyStick.isFixHeight = 0; /* 将固定定高的标志位置为0 */
                isFixHeight          = Com_FAIL;
            }
            else
            {

                /* 由于高度变动时间约20ms, 这里取每5次计算一次pid */
                static uint8_t cnt = 0;
                cnt++;
                if(cnt < HEIGHT_PID_DIVIDER) return;
                cnt = 0;
                dt *= HEIGHT_PID_DIVIDER;
                /* 求z轴速度: 互补滤波 */
                float zSpeed = 0.9f * (zSpeedPID.measure + (Common_IMU_GetNormAccZ() - staticAcc) * dt) +
                               0.1f * (height - heightPID.measure) / dt;
                /*
                    串级pid
                        外环  高度环
                        内环  z轴加速度环
                 */
                heightPID.desire  = heightHold;
                heightPID.measure = height;
                heightPID.dt      = dt;

                zSpeedPID.measure = zSpeed;
                zSpeedPID.dt      = dt;
                Com_PID_CascadePID(&heightPID, &zSpeedPID);
            }
            break;
        }
        default:
            break;
    }
}

/**
 * @description: 把定高的pid赋值到motor上
 * @param {Com_Status} isRemoteUnlocked
 * @return {*}
 */
void App_Flight_MotorWithHeightPID(Com_Status isRemoteUnlocked)
{
    int16_t zPid = LIMIT(zSpeedPID.result, HEIGHT_PID_Z_LIMIT_LOW, HEIGHT_PID_Z_LIMIT_HIGH);
    motorLeftTop.speed     = LIMIT(motorLeftTop.speed + zPid, 0, MOTOR_OUTPUT_LIMIT);
    motorLeftBottom.speed  = LIMIT(motorLeftBottom.speed + zPid, 0, MOTOR_OUTPUT_LIMIT);
    motorRightTop.speed    = LIMIT(motorRightTop.speed + zPid, 0, MOTOR_OUTPUT_LIMIT);
    motorRightBottom.speed = LIMIT(motorRightBottom.speed + zPid, 0, MOTOR_OUTPUT_LIMIT);
}

/**
 * @description: 使飞控工作的核心函数
 * @param {Com_Status} isRemoteUnlock
 * @return {*}
 */
void App_Flight_Work(Com_Status isRemoteUnlock)
{
    if(isRemoteUnlock != Com_OK || joyStick.THR <= MOTOR_MIN_THR)
    {
        motorLeftTop.speed =
            motorLeftBottom.speed =
                motorRightTop.speed =
                    motorRightBottom.speed = 0;
    }

    Inf_Motor_AllMotorsWork();
}
