#include "App_DataProcess.h"

/**
 * @description: 数据处理模块启动
 * @return {*}
 */
void App_DataProcess_Start(void)
{
    Inf_JoyStickAndKey_Init();
}

/**
 * @description: 转换摇杆数据的极性和范围
 * @return {*}
 */
static void App_DataProcess_JoyStickPolarityAndRange(void)
{
    /* 1. 反向电压   [4095, 0]  => [0, 1000]
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
 * @description: 摇杆死区处理 (防止中位漂移导致无人机误动)
 * @return {*}
 */
static void App_DataProcess_JoyStickDeadzone(void)
{
    /* 对PIT/ROL/YAW三个方向应用中位死区 */
    if (abs(joyStick.PIT - JOYSTICK_MIDPOINT) <= JOYSTICK_DEADZONE)
    {
        joyStick.PIT = JOYSTICK_MIDPOINT;
    }
    if (abs(joyStick.ROL - JOYSTICK_MIDPOINT) <= JOYSTICK_DEADZONE)
    {
        joyStick.ROL = JOYSTICK_MIDPOINT;
    }
    if (abs(joyStick.YAW - JOYSTICK_MIDPOINT) <= JOYSTICK_DEADZONE)
    {
        joyStick.YAW = JOYSTICK_MIDPOINT;
    }
    /* THR方向: 低于死区的值视为0 (防止油门中位误触发) */
    if (joyStick.THR <= JOYSTICK_THR_DEADZONE)
    {
        joyStick.THR = 0;
    }
}

/**
 * @description: 对摇杆数据做校准
 * @return {*}
 */
static void App_DataProcess_JoystickWithBias(void)
{
    /* 减去偏置量 */
    joyStick.THR -= joyStickBias.THR;
    joyStick.PIT -= joyStickBias.PIT;
    joyStick.ROL -= joyStickBias.ROL;
    joyStick.YAW -= joyStickBias.YAW;

    /* 在校准后限制边界范围 */
    joyStick.THR = LIMIT(joyStick.THR, JOYSTICK_RANGE_MIN, JOYSTICK_RANGE_MAX);
    joyStick.PIT = LIMIT(joyStick.PIT, JOYSTICK_RANGE_MIN, JOYSTICK_RANGE_MAX);
    joyStick.ROL = LIMIT(joyStick.ROL, JOYSTICK_RANGE_MIN, JOYSTICK_RANGE_MAX);
    joyStick.YAW = LIMIT(joyStick.YAW, JOYSTICK_RANGE_MIN, JOYSTICK_RANGE_MAX);
}

/**
 * @description: 处理摇杆数据 (完整流程)
 * @return {*}
 */
void App_DataProcess_JoyStickDataProcess(void)
{
    taskENTER_CRITICAL();
    /* 1. 扫描摇杆 */
    Inf_JoyStickAndKey_JoyStickScan();
    /* 2. 极性和范围转换 */
    App_DataProcess_JoyStickPolarityAndRange();
    /* 3. 对摇杆数据进行偏差校准 */
    App_DataProcess_JoystickWithBias();
    /* 4. 摇杆死区处理 (防中位漂移) */
    App_DataProcess_JoyStickDeadzone();
    taskEXIT_CRITICAL();
}

/**
 * @description: 计算摇杆的偏置量
 * @return {*}
 */
static void App_DataProcess_JoyStickCaclBias(void)
{
    joyStickBias.THR = 0;
    joyStickBias.ROL = 0;
    joyStickBias.YAW = 0;
    joyStickBias.PIT = 0;

    for(uint8_t i = 0; i < JOYSTICK_BIAS_SAMPLES; i++)
    {
        Inf_JoyStickAndKey_JoyStickScan();
        App_DataProcess_JoyStickPolarityAndRange();
        joyStickBias.THR += (joyStick.THR - 0);   /* 0值校准 */
        joyStickBias.PIT += (joyStick.PIT - JOYSTICK_MIDPOINT); /* 中值校准 */
        joyStickBias.YAW += (joyStick.YAW - JOYSTICK_MIDPOINT);
        joyStickBias.ROL += (joyStick.ROL - JOYSTICK_MIDPOINT);
        vTaskDelay(KEY_DEBOUNCE_MS / 3); /* 约10ms间隔 */
    }

    joyStickBias.THR /= JOYSTICK_BIAS_SAMPLES;
    joyStickBias.PIT /= JOYSTICK_BIAS_SAMPLES;
    joyStickBias.ROL /= JOYSTICK_BIAS_SAMPLES;
    joyStickBias.YAW /= JOYSTICK_BIAS_SAMPLES;
}

/**
 * @description: 按键的处理
 *  按键扫描底层已有30ms软件防抖 (vTaskDelay(30)),
 *  此处为上层按键功能分发逻辑。
 * @return {*}
 */
void App_DataProcess_KeyDataProcess(void)
{
    Com_Key key = Inf_JoyStickAndKey_KeyScan();
    switch(key)
    {
        case KEY_RIGHT_TOP_LONG:
        {
            /* 校准时暂停, 摇杆数据暂不进行处理 */
            taskENTER_CRITICAL();
            App_DataProcess_JoyStickCaclBias();
            taskEXIT_CRITICAL();
            break;
        }
        case KEY_LEFT_TOP:
        {
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
