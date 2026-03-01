#include "App_DataProcess.h"

/**
 * @description: 数据处理模块的启动
 * @return {*}
 */
void App_DataProcess_Start(void)
{
    Inf_JoyStickAndKey_Init();
}

/**
 * @description: 处理摇杆数据的极性和范围
 * @return {*}
 */
static void App_DataProcess_JoyStickPolarityAndRange(void)
{
    /* 1. 处理极性   [4095, 0]  => [0, 1000]
          (4095 -  [4095, 0]) / 4.095
          (4095 -  [4095, 0]) / (4095 / 1000)
          (4095 -  [4095, 0]) * 1000 / 4095
          1000 - [4095, 0] * 1000 / 4095

     */
    joyStick.THR = 1000 - joyStick.THR * 1000 / 4095;
    joyStick.ROL = 1000 - joyStick.ROL * 1000 / 4095;
    joyStick.PIT = 1000 - joyStick.PIT * 1000 / 4095;
    joyStick.YAW = 1000 - joyStick.YAW * 1000 / 4095;
}

/**
 * @description: 对摇杆数据做校准
 * @return {*}
 */
static void App_DataProcess_JoystickWithBias(void)
{
    /* 叠加偏移量 */
    joyStick.THR -= joyStickBias.THR;
    joyStick.PIT -= joyStickBias.PIT;
    joyStick.ROL -= joyStickBias.ROL;
    joyStick.YAW -= joyStickBias.YAW;

    /* 对校准后的数据做限幅处理 */
    joyStick.THR = LIMIT(joyStick.THR, 0, 1000);
    joyStick.PIT = LIMIT(joyStick.PIT, 0, 1000);
    joyStick.ROL = LIMIT(joyStick.ROL, 0, 1000);
    joyStick.YAW = LIMIT(joyStick.YAW, 0, 1000);
}

/**
 * @description: 处理摇杆数据
 * @return {*}
 */
void App_DataProcess_JoyStickDataProcess(void)
{
    taskENTER_CRITICAL();
    /* 1. 扫描摇杆 */
    Inf_JoyStickAndKey_JoyStickScan();
    /* 2. 极性和范围处理 */
    App_DataProcess_JoyStickPolarityAndRange();
    /* 3. 对摇杆数据做数据校准 */
    App_DataProcess_JoystickWithBias();
    taskEXIT_CRITICAL();

    // Com_Config_PrintJoyStick("2");
}

/**
 * @description: 计算摇杆的偏移量
 * @return {*}
 */
static void App_DataProcess_JoyStickCaclBias(void)
{
    joyStickBias.THR = 0;
    joyStickBias.ROL = 0;
    joyStickBias.YAW = 0;
    joyStickBias.PIT = 0;

    for(uint8_t i = 0; i < 100; i++)
    {
        Inf_JoyStickAndKey_JoyStickScan();
        App_DataProcess_JoyStickPolarityAndRange();
        joyStickBias.THR += (joyStick.THR - 0);   /* 0值校准 */
        joyStickBias.PIT += (joyStick.PIT - 500); /* 中值校准 */
        joyStickBias.YAW += (joyStick.YAW - 500);
        joyStickBias.ROL += (joyStick.ROL - 500);
        vTaskDelay(10);
    }

    joyStickBias.THR /= 100;
    joyStickBias.PIT /= 100;
    joyStickBias.ROL /= 100;
    joyStickBias.YAW /= 100;
    // Com_Config_PrintJoyStickBias("bias 2");
}

/**
 * @description: 按键的处理
 * @return {*}
 */
void App_DataProcess_KeyDataProcess(void)
{
    Com_Key key = Inf_JoyStickAndKey_KeyScan();
    switch(key)
    {
        case KEY_RIGHT_TOP_LONG:
        {
            /* 在做校准的时候, 摇杆处理任务不进行处理 */
            taskENTER_CRITICAL();
            App_DataProcess_JoyStickCaclBias();
            taskEXIT_CRITICAL();
            break;
        }
        case KEY_LEFT_TOP:
        {
            debug_printfln("1");
            joyStick.isPowerDown = 1;
            break;
        }
        case KEY_RIGHT_TOP:
        {
            /* 定高 */
            joyStick.isFixHeight = 1;
            break;
        }
        /* 微调按钮 */
        case KEY_LEFT:
        {
            joyStickBias.ROL += 10;
            break;
        }
        case KEY_UP:
        {
            joyStickBias.PIT -= 10;
            break;
        }
        case KEY_RIGHT:
        {
            joyStickBias.ROL -= 10;
            break;
        }
        case KEY_DOWN:
        {
            joyStickBias.PIT += 10;
            break;
        }

        default:
            break;
    }
}

