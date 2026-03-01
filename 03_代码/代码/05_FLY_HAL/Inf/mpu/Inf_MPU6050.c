#include "Inf_MPU6050.h"

#define abs(x) ((x) >= 0 ? (x) : -(x))

static GyroAccel_Struct offsetGyroAccel;

void Inf_MPU6050_Calibrate(void);
/**
 * @description: 向指定的寄存器写入一个字节
 * @param {uint8_t} reg
 * @param {uint8_t} byte
 * @return {*}
 */
void Inf_MPU6050_WriteReg(uint8_t reg, uint8_t byte)
{
    HAL_I2C_Mem_Write(&hi2c1,
                      MPU6050_ADDR_W,
                      reg,
                      I2C_MEMADD_SIZE_8BIT,
                      &byte,
                      1,
                      2000);
}

/**
 * @description: 从指定的寄存器startReg写入多个字节
 * @param {uint8_t} startReg 开始寄存器地址
 * @param {uint8_t} *bytes 写的字节数据
 * @param {uint8_t} len 多少个字节
 * @return {*}
 */
void Inf_MPU6050_WriteRegs(uint8_t startReg, uint8_t *bytes, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c1,
                      MPU6050_ADDR_W,
                      startReg,
                      I2C_MEMADD_SIZE_8BIT,
                      bytes,
                      len,
                      2000);
}

/**
 * @description: 从指定的寄存器读取一个字节
 * @param {uint8_t} reg
 * @return {*}
 */
uint8_t Inf_MPU6050_ReadReg(uint8_t reg)
{
    uint8_t rByte = 0;
    HAL_I2C_Mem_Read(&hi2c1,
                     MPU6050_ADDR_R,
                     reg,
                     I2C_MEMADD_SIZE_8BIT,
                     &rByte,
                     1,
                     2000);
    return rByte;
}

/**
 * @description: 从指定的寄存器读取多个字节
 * @param {uint8_t} startReg
 * @param {uint8_t} *bytes
 * @param {uint8_t} len
 * @return {*}
 */
void Inf_MPU6050_ReadRegs(uint8_t startReg, uint8_t *bytes, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c1,
                     MPU6050_ADDR_R,
                     startReg,
                     I2C_MEMADD_SIZE_8BIT,
                     bytes,
                     len,
                     2000);
}

/**
 * @description: mpu6050初始化函数
 * @return {*}
 */
void Inf_MPU6050_Init(void)
{
    /* 1. 复位MPU6050:   复位->休眠200ms->唤醒 */
    Inf_MPU6050_WriteReg(MPU_PWR_MGMT1_REG, 1 << 7);
    HAL_Delay(200);
    Inf_MPU6050_WriteReg(MPU_PWR_MGMT1_REG, 0);
    /* 2. 设置量程 */
    /* 2.1 角速度的量程  +-2000°/s */
    Inf_MPU6050_WriteReg(MPU_GYRO_CFG_REG, 3 << 3);
    /* 2.2 加速度的量程  +2g  */
    Inf_MPU6050_WriteReg(MPU_ACCEL_CFG_REG, 0);
    /* 3. 关闭中断、关闭 第2 IIC 接口、禁止 FIFO */
    Inf_MPU6050_WriteReg(MPU_INT_EN_REG, 0);
    Inf_MPU6050_WriteReg(MPU_USER_CTRL_REG, 0);
    Inf_MPU6050_WriteReg(MPU_FIFO_EN_REG, 0);

    /* 4. 设置陀螺仪采样率和低通滤波器
        低通滤波器:  数据输出速率时 1kHz

        采样率: 采样率是多少的时候,才能正确的反应模拟信号
            香浓定律(奈奎斯特定律): 采样率 >= 2 * 信号中最高的频率

            采样频率 = 陀螺仪输出频率 / (1 + SMPLRT_DIV)
            500Hz = 1000Hz / (1 + x)
            x = 1
     */

    Inf_MPU6050_WriteReg(MPU_SAMPLE_RATE_REG, 1); /*  采样率 */
    Inf_MPU6050_WriteReg(MPU_CFG_REG, 1 << 0);    /* 低通滤波器 */

    /* 5. 配置系统时钟源 */
    Inf_MPU6050_WriteReg(MPU_PWR_MGMT1_REG, 1);

    /* 6. 使能角速度传感器和加速度传感器   进入工作模式 (禁用待机模式)*/
    Inf_MPU6050_WriteReg(MPU_PWR_MGMT2_REG, 0);

    /* 校准6轴数据 */
    Inf_MPU6050_Calibrate();
}

/**
 * @description: 读取角速度
 * @param {Gyro_Struct} *gyro
 * @return {*}
 */
void Inf_MPU6050_ReadGyro(Gyro_Struct *gyro)
{
    uint8_t data[6] = {0};
    Inf_MPU6050_ReadRegs(MPU_GYRO_XOUTH_REG, data, 6);

    gyro->gyroX = (int16_t)((data[0] << 8) | data[1]);
    gyro->gyroY = (int16_t)((data[2] << 8) | data[3]);
    gyro->gyroZ = (int16_t)((data[4] << 8) | data[5]);
}

/**
 * @description: 读取加速度
 * @param {Accel_Struct} *accel
 * @return {*}
 */
void Inf_MPU6050_ReadAccel(Accel_Struct *accel)
{
    uint8_t data[6] = {0};
    Inf_MPU6050_ReadRegs(MPU_ACCEL_XOUTH_REG, data, 6);
    accel->accelX = (int16_t)((data[0] << 8) | data[1]);
    accel->accelY = (int16_t)((data[2] << 8) | data[3]);
    accel->accelZ = (int16_t)((data[4] << 8) | data[5]);
}

/**
 * @description: 读取角速度和加速度
 * @param {GyroAccel_Struct} *gyroAccel
 * @return {*}
 */
void Inf_MPU6050_ReadGyroAccel(GyroAccel_Struct *gyroAccel)
{
    Inf_MPU6050_ReadGyro(&gyroAccel->gyro);
    Inf_MPU6050_ReadAccel(&gyroAccel->accel);
}

void Inf_MPU6050_ReadGyroAccelCalibrated(GyroAccel_Struct *gyroAccel)
{
    /* 获取原始数据 */
    Inf_MPU6050_ReadGyroAccel(gyroAccel);

    /* 减去校准值 */
    gyroAccel->gyro.gyroX -= offsetGyroAccel.gyro.gyroX;
    gyroAccel->gyro.gyroY -= offsetGyroAccel.gyro.gyroY;
    gyroAccel->gyro.gyroZ -= offsetGyroAccel.gyro.gyroZ;
    gyroAccel->accel.accelX -= offsetGyroAccel.accel.accelX;
    gyroAccel->accel.accelY -= offsetGyroAccel.accel.accelY;
    gyroAccel->accel.accelZ -= offsetGyroAccel.accel.accelZ;
}

/**
 * @description: 在水平平面静止状态下校准6轴数据
 *  1.确认飞机处于静止
 *      多次测量3轴数据(角速度),本次与上次对比,如果变化小于某个阈值,则认为飞机处于静止状态
 *  2.多次测量计算偏移
 *
 *
 * @return {*}
 */
void Inf_MPU6050_Calibrate(void)
{
    /* 1.确认静止  */
    uint8_t          cnt = 30;
    GyroAccel_Struct current;
    GyroAccel_Struct last;
    debug_printfln(" 陀螺仪: 等待静止");
    while(cnt)
    {
        Inf_MPU6050_ReadGyroAccel(&current);
        if(abs(current.gyro.gyroX - last.gyro.gyroX) <= 10 &&
           abs(current.gyro.gyroY - last.gyro.gyroY) <= 10 &&
           abs(current.gyro.gyroZ - last.gyro.gyroZ) <= 10)
        {
            cnt--;
        }

        last = current; /* 上次的结构体执行这次 */
        HAL_Delay(3);
    }
    debug_printfln(" 陀螺仪: 已经静止");

    debug_printfln(" 陀螺仪: 开始校准");
    int32_t  sumBuff[6] = {0};
    uint16_t sumCount   = 255;
    for(uint16_t i = 0; i < sumCount; i++)
    {
        Inf_MPU6050_ReadGyroAccel(&gyroAccel);
        sumBuff[0] += gyroAccel.gyro.gyroX - 0;
        sumBuff[1] += gyroAccel.gyro.gyroY - 0;
        sumBuff[2] += gyroAccel.gyro.gyroZ - 0;
        sumBuff[3] += gyroAccel.accel.accelX - 0;
        sumBuff[4] += gyroAccel.accel.accelY - 0;
        sumBuff[5] += gyroAccel.accel.accelZ - 16383;

        HAL_Delay(3);
    }

    offsetGyroAccel.gyro.gyroX   = sumBuff[0] / sumCount;
    offsetGyroAccel.gyro.gyroY   = sumBuff[1] / sumCount;
    offsetGyroAccel.gyro.gyroZ   = sumBuff[2] / sumCount;
    offsetGyroAccel.accel.accelX = sumBuff[3] / sumCount;
    offsetGyroAccel.accel.accelY = sumBuff[4] / sumCount;
    offsetGyroAccel.accel.accelZ = sumBuff[5] / sumCount;

    // Com_Config_PrintGyroAccel("cal", &offsetGyroAccel);
    debug_printfln(" 陀螺仪: 结束校准");
}
