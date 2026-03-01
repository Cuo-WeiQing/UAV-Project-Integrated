#include "App_Flight.h"

/* 6个姿态pid */
/* 俯仰 */
PID_Struct pitchPID = {.kp = -7.0f, .ki = 0.0f, .kd = 0.0f};
PID_Struct gyroYPID = {.kp = 2.0f, .ki = 0.0f, .kd = 0.1f};

/* 横滚 */
PID_Struct rollPID  = {.kp = -7.0f, .ki = 0.0f, .kd = 0.0f};
PID_Struct gyroXPID = {.kp = -2.0f, .ki = 0.0f, .kd = -0.1f};

/* 偏航 */
PID_Struct yawPID   = {.kp = -2.2f, .ki = 0.0f, .kd = 0.0f};
PID_Struct gyroZPID = {.kp = -1.5f, .ki = 0.0f, .kd = 0.0f};

/* 2个定高pid */
/* 高度 */
PID_Struct heightPID = {.kp = -1.4f, .ki = 0.0f, .kd = 0.0f};
PID_Struct zSpeedPID = {.kp = -1.3f, .ki = 0.0f, .kd = -0.08f};
/**
 * @description: 飞行模块的启动
 * @return {*}
 */
void App_Flight_Start(void)
{
    /* 1. 初始化motor */
    debug_printfln("初始化马达 开始");
    Inf_Motor_Init();
    debug_printfln("初始化马达 结束");

    /* 2. 初始化MPU6050 */
    debug_printfln("初始化MPU6050 开始");
    Inf_MPU6050_Init();
    debug_printfln("初始化MPU6050 结束");

    /* 3. 初始化VL53L1X */
    debug_printfln("初始化激光测距 开始");
    Inf_VL53LX1_Init();
    debug_printfln("初始化激光测距 结束");
}

/**
 * @description: 对六轴数据做滤波
 *  角速度: 使用一阶低通滤波
 *      波动小, 外界影响小
 *  加速度: 卡尔曼滤波
 *      波动大, 容易收到干扰
 * @param {GyroAccel_Struct} *gyroAccel
 * @return {*}
 */
void App_Flight_GetGyroAccelWithFilter(GyroAccel_Struct *gyroAccel)
{
    /* 1. 读取原始数据 */
    taskENTER_CRITICAL();
    Inf_MPU6050_ReadGyroAccelCalibrated(gyroAccel);
    taskEXIT_CRITICAL();

    /* 2. 对角速度做一阶低通低通滤波 */
    static int16_t lastDatas[3] = {0};
    gyroAccel->gyro.gyroX       = Com_Filter_LowPass(gyroAccel->gyro.gyroX, lastDatas[0]);
    gyroAccel->gyro.gyroY       = Com_Filter_LowPass(gyroAccel->gyro.gyroY, lastDatas[1]);
    gyroAccel->gyro.gyroZ       = Com_Filter_LowPass(gyroAccel->gyro.gyroZ, lastDatas[2]);
    lastDatas[0]                = gyroAccel->gyro.gyroX;
    lastDatas[1]                = gyroAccel->gyro.gyroY;
    lastDatas[2]                = gyroAccel->gyro.gyroZ;

    /* 3. 对加速度做卡尔曼滤波 */
    // OutData[0] = gyroAccel->accel.accelX ;
    gyroAccel->accel.accelX = Common_Filter_KalmanFilter(&kfs[0], gyroAccel->accel.accelX);
    gyroAccel->accel.accelY = Common_Filter_KalmanFilter(&kfs[1], gyroAccel->accel.accelY);
    gyroAccel->accel.accelZ = Common_Filter_KalmanFilter(&kfs[2], gyroAccel->accel.accelZ);
    // OutData[1] = gyroAccel->accel.accelX ;
    // OutPut_Data();
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
 * @param {float} dt 调度周期
 * @return {*}
 */
void App_Flight_PIDPosture(GyroAccel_Struct *gyroAccel, EulerAngle_Struct *eulerAngle, float dt)
{
    /* 俯仰 */
    pitchPID.dt      = dt;
    pitchPID.desire  = (joyStick.PIT - 500) * 0.04; /* 控制飞行 */
    pitchPID.measure = eulerAngle->pitch;

    gyroYPID.dt      = dt;
    gyroYPID.measure = gyroAccel->gyro.gyroY * Gyro_G;

    Com_PID_CascadePID(&pitchPID, &gyroYPID);

    /* 横滚 */
    rollPID.dt      = dt;
    rollPID.desire  = (joyStick.ROL - 500) * 0.04;
    rollPID.measure = eulerAngle->roll;

    gyroXPID.dt      = dt;
    gyroXPID.measure = gyroAccel->gyro.gyroX * Gyro_G;

    Com_PID_CascadePID(&rollPID, &gyroXPID);

    /* 偏航 */
    yawPID.dt      = dt;
    yawPID.desire  = (joyStick.YAW - 500) * 0.04;
    yawPID.measure = eulerAngle->yaw;

    gyroZPID.dt      = dt;
    gyroZPID.measure = gyroAccel->gyro.gyroZ * Gyro_G;

    Com_PID_CascadePID(&yawPID, &gyroZPID);
}

/**
 * @description: 把姿态PID作用到电机上
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
    int16_t speed          = joyStick.THR * 0.7;
    motorLeftTop.speed     = speed + gyroXPID.result + gyroYPID.result + gyroZPID.result;
    motorLeftBottom.speed  = speed + gyroXPID.result - gyroYPID.result - gyroZPID.result;
    motorRightTop.speed    = speed - gyroXPID.result + gyroYPID.result - gyroZPID.result;
    motorRightBottom.speed = speed - gyroXPID.result - gyroYPID.result + gyroZPID.result;
}

/**
 * @description: 获取飞机的飞行高度
 * @return {*} 高度: mm
 */
uint16_t App_Flight_GetHeight(void)
{
    static uint16_t lastHeight = 0;
    uint16_t        height     = Inf_VL53LX1_GetHeight();

    if(abs(height - lastHeight) > 500 || /* 如果有突变,则返回上次的值 */
       abs(joyStick.PIT - 500) > 100 ||  /* 有水平飞行, 返回上次的值 */
       abs(joyStick.ROL - 500) > 100)
    {
        return lastHeight;
    }

    height     = Com_Filter_LowPass(height, lastHeight);
    lastHeight = height;

    return height;
}

/**
 * @description: 高度pid控制
 * @param {Com_Status} isRemoteUnlocked
 * @param {uint16_t} height
 * @return {*}
 */
void App_Flight_PIDHeight(Com_Status isRemoteUnlocked, uint16_t height, float dt)
{
    /* 定高状态机:
        状态0: 检测是否定高
        状态1: 当前的油门值是定高时的油门值  当前的高度: 固定的高度
        状态2: 进行pid控制
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
            /* pid重置 */
            heightPID.result = 0;
            zSpeedPID.result = 0;
            if(isRemoteUnlocked == Com_OK && isFixHeight == Com_OK)
            {
                status = 1;
            }

            break;
        }

        case 1: /* pid计算前的准备 */
        {
            thrHold    = joyStick.THR;
            heightHold = height;
            status     = 2;
            break;
        }
        case 2: /* pid计算 */
        {
            /* 定高时: 油门变化超过100, 或者定高的标记为0. 解除定高 */
            if(abs(joyStick.THR - thrHold) > 100 || isFixHeight == Com_FAIL)
            {
                status               = 0; /* 回到状态0 */
                joyStick.isFixHeight = 0; /* 标记定高的变量置为0 */
                isFixHeight          = Com_FAIL;
            }
            else
            {

                /* 由于高度变动的周期20ms, 所以我们需要5次来计算一次pid */
                static uint8_t cnt = 0;
                cnt++;
                if(cnt < 5) return;
                cnt = 0;
                dt *= 5;
                /* 对z的速度: 互补滤波 */
                float zSpeed = 0.9 * (zSpeedPID.measure + (Common_IMU_GetNormAccZ() - staticAcc) * dt) +
                               0.1 * (height - heightPID.measure) / dt;
                /*
                    串级pid
                        外环  高度环
                        内环  z方向的速度环
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
 * @description: 把定高的pid作用到motor上
 * @param {Com_Status} isRemoteUnlocked
 * @return {*}
 */
void App_Flight_MotorWithHeightPID(Com_Status isRemoteUnlocked)
{
    int16_t zPid = LIMIT(zSpeedPID.result, -150, 150);
    motorLeftTop.speed += zPid;
    motorLeftBottom.speed += zPid;
    motorRightTop.speed += zPid;
    motorRightBottom.speed += zPid;
}

/**
 * @description: 让飞机工作(飞行)
 * @param {Com_Status} isRemoteUnlock
 * @return {*}
 */
void App_Flight_Work(Com_Status isRemoteUnlock)
{
    if(isRemoteUnlock != Com_OK || joyStick.THR <= 30)
    {
        motorLeftTop.speed =
            motorLeftBottom.speed =
                motorRightTop.speed =
                    motorRightBottom.speed = 0;
    }

    Inf_Motor_AllMotorsWork();
}

/*
模拟信号滤波
    电容  电感

数字信号滤波
    算法
        1. 均值滤波(滑动窗口滤波)
        2. 中值滤波
        3. 一阶低通滤波
             结果 = 系数 * 上次的值 + (1 - 系数) * 这次的值
             角速度

        4. 卡尔曼滤波
            核心5个公式
                https://www.kalmanfilter.net/CN/default_cn.aspx

            加速度

        5. 互补滤波
            例子:
                加速度对时间积分    速度: 响应迅速, 结果容易收到外界影响
                位移对时间的微分    速度: 结果不容受影响, 响应不及时

                0.3 * 速度1 + 0.7 * 速度2

        ...


如何调节pid:

    内环 p
        确定极性:
            错误: 加速下跌
            正确: 缓慢平滑的下跌
        大小:
            下跌的时候要平滑
    外环 p
        确定极性:
            正确: 回到初始位置
            错误: 旋转
        大小:
            观察回来的快慢. 看是否有震荡, 有适量的震荡
    内环 d
        抑制震荡
        确定极性:
            错误: 震荡加剧
            正确: 震荡减弱
        大小:
            既能快速回到初始位置,还没有震荡
    外环 d(可选)
        如果内环的d抑制不了震荡,再调节外环的d
    内环 i (可选)
    外环 i (可选)



*/
