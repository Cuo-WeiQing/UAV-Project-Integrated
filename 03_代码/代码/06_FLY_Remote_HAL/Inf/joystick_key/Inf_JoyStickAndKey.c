#include "Inf_JoyStickAndKey.h"

#define READ_LEFT HAL_GPIO_ReadPin(KEY_LEFT_GPIO_Port, KEY_LEFT_Pin)
#define READ_UP HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin)
#define READ_RIGHT HAL_GPIO_ReadPin(KEY_RIGHT_GPIO_Port, KEY_RIGHT_Pin)
#define READ_DOWN HAL_GPIO_ReadPin(KEY_DOWN_GPIO_Port, KEY_DOWN_Pin)
#define READ_LEFT_TOP HAL_GPIO_ReadPin(KEY_LEFT_TOP_GPIO_Port, KEY_LEFT_TOP_Pin)
#define READ_RIGHT_TOP HAL_GPIO_ReadPin(KEY_RIGHT_TOP_GPIO_Port, KEY_RIGHT_TOP_Pin)

/* 按键防抖与超时参数 */
#define KEY_DEBOUNCE_MS         (30)    /* 软件防抖延时 (ms) */
#define KEY_SHORT_PRESS_MAX     (5)     /* 短按最大时间单位 (x100ms) */
#define KEY_LONG_PRESS_TIMEOUT  (12)    /* 长按超时检测单位 (x100ms) */
#define KEY_RELEASE_TIMEOUT     (100)   /* 等待按键释放的超时次数 */

static uint16_t buff[4] = {0};

/**
 * @description: 摇杆初始化
 * @return {*}
 */
void Inf_JoyStickAndKey_Init(void)
{
    debug_printfln("摇杆和按键数据的初始化 开始");
    /* 1. ADC校准 */
    HAL_ADCEx_Calibration_Start(&hadc1);

    /* 2. 启动ADC转换 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)buff, 4);
    debug_printfln("摇杆和按键数据的初始化 结束");
}

/**
 * @description: 扫描摇杆数据
 * @return {*}
 */
void Inf_JoyStickAndKey_JoyStickScan(void)
{
    joyStick.THR = buff[0];
    joyStick.YAW = buff[1];
    joyStick.PIT = buff[2];
    joyStick.ROL = buff[3];
}

/**
 * @description: 等待按键释放 (带超时保护，防止卡键死循环)
 * @param {GPIO_TypeDef*} port GPIO端口
 * @param {uint16_t} pin GPIO引脚
 * @return {*}
 */
static void Inf_JoyStickAndKey_WaitRelease(GPIO_TypeDef *port, uint16_t pin)
{
    uint16_t timeout = 0;
    while (HAL_GPIO_ReadPin(port, pin) == 0 && timeout < KEY_RELEASE_TIMEOUT)
    {
        timeout++;
        vTaskDelay(10);
    }
}

/**
 * @description: 扫描按键 (带软件防抖、长按检测、卡键超时保护)
 * @return {*} 按下的那个按键
 */
Com_Key Inf_JoyStickAndKey_KeyScan(void)
{
    if(READ_LEFT == 0 ||
       READ_UP == 0 ||
       READ_DOWN == 0 ||
       READ_RIGHT == 0 ||
       READ_LEFT_TOP == 0 ||
       READ_RIGHT_TOP == 0)
    {
        /* 软件防抖: 延时后再次确认按键状态 */
        vTaskDelay(KEY_DEBOUNCE_MS);

        if(READ_LEFT == 0)
        {
            Inf_JoyStickAndKey_WaitRelease(KEY_LEFT_GPIO_Port, KEY_LEFT_Pin);
            return KEY_LEFT;
        }
        else if(READ_RIGHT == 0)
        {
            Inf_JoyStickAndKey_WaitRelease(KEY_RIGHT_GPIO_Port, KEY_RIGHT_Pin);
            return KEY_RIGHT;
        }
        else if(READ_UP == 0)
        {
            Inf_JoyStickAndKey_WaitRelease(KEY_UP_GPIO_Port, KEY_UP_Pin);
            return KEY_UP;
        }
        else if(READ_DOWN == 0)
        {
            Inf_JoyStickAndKey_WaitRelease(KEY_DOWN_GPIO_Port, KEY_DOWN_Pin);
            return KEY_DOWN;
        }
        else if(READ_LEFT_TOP == 0)
        {
            uint16_t time = 0;
            /* 长按检测: 每100ms检测一次, 最长1.2s */
            while(READ_LEFT_TOP == 0 && time < KEY_LONG_PRESS_TIMEOUT)
            {
                time++;
                vTaskDelay(100);
            }
            if(time <= KEY_SHORT_PRESS_MAX)
            {
                return KEY_LEFT_TOP;
            }
            /* 长按: 等待释放后返回 */
            Inf_JoyStickAndKey_WaitRelease(KEY_LEFT_TOP_GPIO_Port, KEY_LEFT_TOP_Pin);
            return KEY_LEFT_TOP_LONG;
        }
        else if(READ_RIGHT_TOP == 0)
        {
            uint16_t time = 0;
            while(READ_RIGHT_TOP == 0 && time < KEY_LONG_PRESS_TIMEOUT)
            {
                time++;
                vTaskDelay(100);
            }
            if(time <= KEY_SHORT_PRESS_MAX)
            {
                return KEY_RIGHT_TOP;
            }
            /* 长按: 等待释放后返回 */
            Inf_JoyStickAndKey_WaitRelease(KEY_RIGHT_TOP_GPIO_Port, KEY_RIGHT_TOP_Pin);
            return KEY_RIGHT_TOP_LONG;
        }
    }
    return KEY_NONE;
}
