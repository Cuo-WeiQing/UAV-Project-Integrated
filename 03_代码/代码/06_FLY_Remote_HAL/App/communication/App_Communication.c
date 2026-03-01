#include "App_Communication.h"

/**
 * @description: 启动通讯模块
 * @return {*}
 */
void App_Communication_Start(void)
{
    debug_printfln("通讯模块的启动 开始");

    debug_printfln(" 2.4g模块自检 开始");
    while(Inf_Si24R1_Check() == 1)
    {
        HAL_Delay(500);
    }
    debug_printfln(" 2.4g模块自检 结束");

    debug_printfln(" 2.4设置为发送模式");
    Inf_Si24R1_TXMode();

    debug_printfln("通讯模块的启动 结束");
}

/**
 * @description: 通过2.4g发送摇杆数据
 *  前3个字节: 0x01,0x02,0x03
 *  1个字节:   真正的数据长度  = 10
 *  10个字节:  真正的数据
 *  4个字节 :  校验和
 *
 * @return {*}
 */
void App_Communication_SendJoyStickData(void)
{
    uint8_t index = 0;
    /* 定义帧头 */
    TX_BUFF[index++] = FRAME_0;
    TX_BUFF[index++] = FRAME_1;
    TX_BUFF[index++] = FRAME_2;

    /* 定义实际的数据长度  后面根据实际的数据,再修改*/
    TX_BUFF[index++] = 0;

    /* 摇杆数据 */
    TX_BUFF[index++] = joyStick.THR >> 8;
    TX_BUFF[index++] = joyStick.THR;

    TX_BUFF[index++] = joyStick.YAW >> 8;
    TX_BUFF[index++] = joyStick.YAW;

    TX_BUFF[index++] = joyStick.PIT >> 8;
    TX_BUFF[index++] = joyStick.PIT;

    TX_BUFF[index++] = joyStick.ROL >> 8;
    TX_BUFF[index++] = joyStick.ROL;

    TX_BUFF[index++]     = joyStick.isPowerDown;
    joyStick.isPowerDown = 0; /* 清零关机命令 */

    TX_BUFF[index++]     = joyStick.isFixHeight; /* 接收方收到1之后,对定高进行取反操作 */
    joyStick.isFixHeight = 0;                    /* 只发一次 */

    TX_BUFF[3] = index - 4;

    /* 计算校验和 */
    int32_t sum = 0;
    for(uint8_t i = 0; i < index; i++)
    {
        sum += TX_BUFF[i];
    }
    TX_BUFF[index++] = sum >> 24;
    TX_BUFF[index++] = sum >> 16;
    TX_BUFF[index++] = sum >> 8;
    TX_BUFF[index++] = sum;

    taskENTER_CRITICAL();
    Inf_Si24R1_TxPacket(TX_BUFF);
    taskEXIT_CRITICAL();
}
