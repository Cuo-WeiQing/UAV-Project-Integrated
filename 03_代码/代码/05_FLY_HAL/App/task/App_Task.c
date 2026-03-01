#include "App_Task.h"

/* 0. 启动任务 */
void startTask(void *args);
#define START_TASK_NAME "start_task"
#define START_TASK_STACK 128
#define START_TASK_PRIORITY 10
TaskHandle_t startTaskHandle;

/* 1. 电源管理任务 */
void powerTask(void *args);
#define POWER_TASK_NAME "power_task"
#define POWER_TASK_STACK 128
#define POWER_TASK_PRIORITY 9
TaskHandle_t powerTaskHandle;
#define POWER_EXEC_CYCLE (10 * 1000)

/* 2. 灯控任务 */
void ledTask(void *args);
#define LED_TASK_NAME "led_task"
#define LED_TASK_STACK 128
#define LED_TASK_PRIORITY 2
TaskHandle_t ledTaskHandle;
#define LED_EXEC_CYCLE 50

/* 3. 飞控任务 */
void flightTask(void *args);
#define FLIGHT_TASK_NAME "flight_task"
#define FLIGHT_TASK_STACK 256
#define FLIGHT_TASK_PRIORITY 8
TaskHandle_t flightTaskHandle;
#define FLIGHT_EXEC_CYCLE 4

/* 4. 通讯任务 */
void communicationTask(void *args);
#define COMMUNICATION_TASK_NAME "communication_task"
#define COMMUNICATION_TASK_STACK 256
#define COMMUNICATION_TASK_PRIORITY 8
TaskHandle_t communicationTaskHandle;
#define COMMUNICATION_EXEC_CYCLE 6

/**
 * @description: 启动实时系统
 * @return {*}
 */
void App_Task_FreeRTOSStart(void)
{
    /* 1. 初始化debug模块 */
    debug_start();

    debug_printfln("尚硅谷无人机项目--飞控");

    /* 2. 启动飞行模块 */
    App_Flight_Start();

    /* 3. 启动通讯模块 */
    App_Communication_Start();

    /* 1. 创建一个启动任务: 启动任务执行的时候, 创建和业务相关的任务 */
    BaseType_t r = xTaskCreate(startTask,        /* 启动任务函数 */
                               START_TASK_NAME,  /* 任务名 */
                               START_TASK_STACK, /* 栈大小: 单位是字(不是自己) 字的大小: 32位架构是4  START_TASK_STACK * 4 = 字节数  */
                               NULL,             /* 指定给任务函数传递参数 */
                               START_TASK_PRIORITY,
                               &startTaskHandle);
    if(r == pdPASS)
    {
        debug_printfln("启动任务创建成功...");
    }
    else
    {
        debug_printfln("启动任务创建失败...");
    }

    /* 2. 启动调度器 */
    vTaskStartScheduler();
}

/* 启动任务函数 */
void startTask(void *args)
{
    debug_printfln("启动任务开始调度");

    /* 1.创建电源控制任务 */
    xTaskCreate(powerTask,
                POWER_TASK_NAME,
                POWER_TASK_STACK,
                NULL,
                POWER_TASK_PRIORITY,
                &powerTaskHandle);
    /* 2. 创建灯控任务 */
    xTaskCreate(ledTask,
                LED_TASK_NAME,
                LED_TASK_STACK,
                NULL,
                LED_TASK_PRIORITY,
                &ledTaskHandle);

    /* 3. 创建飞控任务 */
    xTaskCreate(flightTask,
                FLIGHT_TASK_NAME,
                FLIGHT_TASK_STACK,
                NULL,
                FLIGHT_TASK_PRIORITY,
                &flightTaskHandle);

    /* 4. 创建通讯任务 */
    xTaskCreate(communicationTask,
                COMMUNICATION_TASK_NAME,
                COMMUNICATION_TASK_STACK,
                NULL,
                COMMUNICATION_TASK_PRIORITY,
                &communicationTaskHandle);

    /* 删除自己 */
    vTaskDelete(NULL);
    debug_printfln("不会执行");
}

/* 电源任务 */
void powerTask(void *args)
{
    debug_printfln("电源任务开始调度");

    uint32_t preTime = xTaskGetTickCount();
    while(1)
    {

        if(ulTaskNotifyTake(pdTRUE, POWER_EXEC_CYCLE))
        {
            /*在 POWER_EXEC_CYCLE 内等到了关机命令 */
            debug_printfln("关机");
            Inf_IP5305T_Close();
        }
        else
        {
            debug_printfln("电源...");
            Inf_IP5305T_Open(); /* 防止自动关机 */
        }
    }
}

/* 2. 灯控任务 */
void ledTask(void *args)
{
    debug_printfln("灯控任务开始调度...");

    // isRemoteConnected = Com_OK;
    // isRemoteUnlocked = Com_OK;
    // isFixHeight = Com_OK;

    uint32_t preTime = xTaskGetTickCount();
    uint32_t cnt     = 0;

    static uint8_t isFirstUnlock = 1; /* 是否时首次进入解锁 */
    while(1)
    {
        /* 连接->解锁->定高 */
        if(isFixHeight == Com_OK)
        {
            ledLeftTop.status =
                ledLeftBootom.status =
                    ledRightTop.status =
                        ledRightBottom.status = 2;
        }
        else if(isRemoteUnlocked == Com_OK)
        {
            if(isFirstUnlock == 1)
            {
                ledLeftTop.status =
                    ledLeftBootom.status =
                        ledRightTop.status =
                            ledRightBottom.status = 1;

                isFirstUnlock = 0;
            }
            else
            {
                ledLeftTop.status =
                    ledLeftBootom.status =
                        ledRightTop.status =
                            ledRightBottom.status = 10;
            }
        }
        else if(isRemoteConnected == Com_OK)
        {
            isFirstUnlock = 1;

            ledLeftTop.status =
                ledRightTop.status = 1;
            ledLeftBootom.status =
                ledRightBottom.status = 15;
        }
        else
        {
            ledLeftTop.status =
                ledRightTop.status = 15;
            ledLeftBootom.status =
                ledRightBottom.status = 1;
        }

        cnt++;

        if(ledLeftTop.status <= 1 || cnt % ledLeftTop.status == 0)
        {
            Inf_LED_SetStatus(&ledLeftTop);
        }

        if(ledLeftBootom.status <= 1 || cnt % ledLeftBootom.status == 0)
        {
            Inf_LED_SetStatus(&ledLeftBootom);
        }

        if(ledRightTop.status <= 1 || cnt % ledRightTop.status == 0)
        {
            Inf_LED_SetStatus(&ledRightTop);
        }

        if(ledRightBottom.status <= 1 || cnt % ledRightBottom.status == 0)
        {
            Inf_LED_SetStatus(&ledRightBottom);
        }
        vTaskDelayUntil(&preTime, LED_EXEC_CYCLE);
    }
}

/* 3. 飞控任务 */
void flightTask(void *args)
{
    debug_printfln("飞控任务开始调度");
    uint32_t preTime = xTaskGetTickCount();

    float dt = FLIGHT_EXEC_CYCLE / 1000.0; /* 调度周期 */
    while(1)
    {

        /* 1. 对6轴数据做滤波 */
        App_Flight_GetGyroAccelWithFilter(&gyroAccel);

        /* 2. 获取欧拉角 */
        App_Flight_GetEulerAngle(&gyroAccel,
                                 &eulerAngle,
                                 dt);

        /* 3. 计算6个pid(3个串级pid) */
        App_Flight_PIDPosture(&gyroAccel, &eulerAngle, dt);

        /* 4. 把姿态pid作用到电机上 */
        App_Flight_MotorWithPosturePID(isRemoteUnlocked);

        /* 5. 获取高度 */
        uint16_t height = App_Flight_GetHeight();

        /* 7. 计算定高pid */
        App_Flight_PIDHeight(isRemoteUnlocked, height, dt);

        /* 8.把高度pid作用到电机上 */
        App_Flight_MotorWithHeightPID(isRemoteUnlocked);
        
        /* 控制电机转动 */
        App_Flight_Work(isRemoteUnlocked);
        vTaskDelayUntil(&preTime, FLIGHT_EXEC_CYCLE);
    }
}

/* 4. 通讯任务 */
void communicationTask(void *args)
{
    // vTaskDelay(1000);
    debug_printfln("通讯任务开始调度");
    uint32_t preTime = xTaskGetTickCount();
    while(1)
    {
        /* 1. 读取2.4g数据 */
        Com_Status isReceiveData = App_Communication_ReceiveJoyStickData();

        /* 2. 判断遥控器的连接情况 */
        isRemoteConnected = App_Communication_CheckConnection(isReceiveData);
        // debug_printfln("%d", isRemoteConnected);

        /* 3. 遥控器解锁 */
        isRemoteUnlocked = App_Communication_RemoteUnlock(isRemoteConnected);

        /* 远程关机 */
        if(isReceiveData == Com_OK)
        {
            if(joyStick.isPowerDown)
            {
                xTaskNotifyGive(powerTaskHandle);
            }
        }
        vTaskDelayUntil(&preTime, COMMUNICATION_EXEC_CYCLE);
    }
}
