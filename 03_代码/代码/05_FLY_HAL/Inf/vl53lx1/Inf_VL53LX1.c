#include "Inf_VL53LX1.h"

#define DEV 0x52
/**
 * @description: 初始化激光测距芯片
 * @return {*}
 */
void Inf_VL53LX1_Init(void)
{
    /* 1. 先重启芯片 */
    HAL_GPIO_WritePin(VL53LX1_SHUT_GPIO_Port, VL53LX1_SHUT_Pin, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(VL53LX1_SHUT_GPIO_Port, VL53LX1_SHUT_Pin, GPIO_PIN_SET);

    /* 2. 初始化化芯片 */
    VL53L1X_SensorInit(DEV);

    /* 3. 设置距离模式: 长或短  1:short 2:long*/
    VL53L1X_SetDistanceMode(DEV, 2);

    /* 4. 测量的速度 */
    VL53L1X_SetTimingBudgetInMs(DEV, 20);

    /* 5. 测量的频率 ms值必须大于等于上一个*/
    VL53L1X_SetInterMeasurementInMs(DEV, 20);

    /* 6. 开始测量 */
    VL53L1X_StartRanging(DEV);

    uint16_t sensorID;
    VL53L1X_GetSensorId(DEV, &sensorID);
    printf("sensorID:0x%x\r\n", sensorID);
}

/**
 * @description: 返回测到的高度
 * @return {*}
 */
uint16_t Inf_VL53LX1_GetHeight(void)
{
    static uint16_t height = 0;
    uint8_t         isDataReady;

    /* 检测测距是否完成 */
    VL53L1X_CheckForDataReady(DEV, &isDataReady);
    if(isDataReady)
    {
        VL53L1X_ClearInterrupt(DEV);
        /* 读取测距结果 */
        VL53L1X_GetDistance(DEV, &height);
    }

    return height;
}

