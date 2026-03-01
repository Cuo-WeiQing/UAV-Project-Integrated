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

/* 2. 通讯任务 */
void communicationTask(void *args);
#define COMMUNICATION_TASK_NAME "communication_task"
#define COMMUNICATION_TASK_STACK 256
#define COMMUNICATION_TASK_PRIORITY 8
TaskHandle_t communicationTaskHandle;
#define COMMUNICATION_EXEC_CYCLE 6

/* 3. 按键处理任务 */
void keyTask(void *args);
#define KEY_TASK_NAME "key_task"
#define KEY_TASK_STACK 256
#define KEY_TASK_PRIORITY 7
TaskHandle_t keyTaskHandle;
#define KEY_EXEC_CYCLE 50

/* 4. 摇杆处理任务 */
void joyStickTask(void *args);
#define JOY_STICK_TASK_NAME "joy_stick_task"
#define JOY_STICK_TASK_STACK 256
#define JOY_STICK_TASK_PRIORITY 7
TaskHandle_t joyStickTaskHandle;
#define JOY_STICK_EXEC_CYCLE 4

/* 5. 显示任务 */
void displayTask(void *args);
#define DISPLAY_TASK_NAME "display_task"
#define DISPLAY_TASK_STACK 256
#define DISPLAY_TASK_PRIORITY 7
TaskHandle_t displayTaskHandle;
#define DISPLAY_EXEC_CYCLE 6

/**
 * @description: 启动实时系统
 * @return {*}
 */
void App_Task_FreeRTOSStart(void)
{
    /* 1. 初始化debug模块 */
    debug_start();

    debug_printfln("尚硅谷无人机项目--遥控");

    /* 2. 启动通讯模块 */
    App_Communication_Start();

    /* 3. 启动处理处理模块 */
    App_DataProcess_Start();

    /* 4. 启动显示模块 */
    App_Display_Start();

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

    /* 2. 创建通讯任务 */
    xTaskCreate(communicationTask,
                COMMUNICATION_TASK_NAME,
                COMMUNICATION_TASK_STACK,
                NULL,
                COMMUNICATION_TASK_PRIORITY,
                &communicationTaskHandle);

    /* 3. 按键处理任务 */
    xTaskCreate(keyTask,
                KEY_TASK_NAME,
                KEY_TASK_STACK,
                NULL,
                KEY_TASK_PRIORITY,
                &keyTaskHandle);

    /* 4. 摇杆处理任务 */
    xTaskCreate(joyStickTask,
                JOY_STICK_TASK_NAME,
                JOY_STICK_TASK_STACK,
                NULL,
                JOY_STICK_TASK_PRIORITY,
                &joyStickTaskHandle);

    /* 5. 显示任务 */
    xTaskCreate(displayTask,
                DISPLAY_TASK_NAME,
                DISPLAY_TASK_STACK,
                NULL,
                DISPLAY_TASK_PRIORITY,
                &displayTaskHandle);

    /* 删除自己 */
    vTaskDelete(NULL);
    debug_printfln("不会执行");
}

/*1 电源任务 */
void powerTask(void *args)
{
    debug_printfln("电源任务开始调度");

    uint32_t preTime = xTaskGetTickCount();
    while(1)
    {

        if(ulTaskNotifyTake(pdTRUE, POWER_EXEC_CYCLE))
        {
            /*在 POWER_EXEC_CYCLE 内等到了关机命令 */
            //debug_printfln("关机");
            Inf_IP5305T_Close();
        }
        else
        {
            //debug_printfln("电源...");
            Inf_IP5305T_Open(); /* 防止自动关机 */
        }
    }
}

/* 2. 通讯任务 */
void communicationTask(void *args)
{
    vTaskDelay(1000);
    debug_printfln("通讯任务开始调度");
    uint32_t preTime = xTaskGetTickCount();

    while(1)
    {
        App_Communication_SendJoyStickData();
        vTaskDelayUntil(&preTime, COMMUNICATION_EXEC_CYCLE);
    }
}

/* 3. 按键扫描 */
void keyTask(void *args)
{
    debug_printfln("按键扫描任务开始调度");
    uint32_t preTime = xTaskGetTickCount();
    while(1)
    {
        App_DataProcess_KeyDataProcess();

        vTaskDelayUntil(&preTime, KEY_EXEC_CYCLE);
    }
}

/* 4. 摇杆处理 */
void joyStickTask(void *args)
{
    vTaskDelay(500);
    debug_printfln("摇杆处理任务开始调度");
    uint32_t preTime = xTaskGetTickCount();
    while(1)
    {
        App_DataProcess_JoyStickDataProcess();
        vTaskDelayUntil(&preTime, JOY_STICK_EXEC_CYCLE);
    }
}

/* 5. 显示任务 */
void displayTask(void *args)
{
    debug_printfln("显示任务开始调度");
    uint32_t preTime = xTaskGetTickCount();
    while(1)
    {
        App_Display_Show();
        vTaskDelayUntil(&preTime, DISPLAY_EXEC_CYCLE);
    }
}
