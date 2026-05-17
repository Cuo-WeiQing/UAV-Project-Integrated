#include "App_Communication.h"

/**
 * @description: 计算接收数据的校验和 (与遥控器端发送的校验算法一致)
 *  校验方式: 从帧头到数据负载末尾的所有字节累加, 取低32位
 * @param {uint8_t} *data 接收缓冲区
 * @param {uint8_t} totalLen 总帧长度 (帧头3 + 长度1 + 数据N + 校验4)
 * @return {int32_t} 计算得到的校验和
 */
static int32_t App_Communication_CalcChecksum(uint8_t *data, uint8_t totalLen)
{
    int32_t sum = 0;
    uint8_t payloadEnd = totalLen - 4; /* 最后4字节是校验和 */
    for (uint8_t i = 0; i < payloadEnd; i++)
    {
        sum += data[i];
    }
    return sum;
}

/**
 * @description: 验证接收帧的校验和
 * @param {uint8_t} *data 接收缓冲区
 * @param {uint8_t} totalLen 总帧长度
 * @return {uint8_t} 1=校验通过 0=校验失败
 */
static uint8_t App_Communication_VerifyChecksum(uint8_t *data, uint8_t totalLen)
{
    int32_t calcSum = App_Communication_CalcChecksum(data, totalLen);
    int32_t recvSum = ((int32_t)data[totalLen - 4] << 24) |
                      ((int32_t)data[totalLen - 3] << 16) |
                      ((int32_t)data[totalLen - 2] << 8)  |
                      ((int32_t)data[totalLen - 1]);
    return (calcSum == recvSum) ? 1 : 0;
}

/**
 * @description: 初始化通讯模块
 * @return {*}
 */
void App_Communication_Start(void)
{
    debug_printfln("通讯模块初始化 开始");

    debug_printfln(" 2.4g模块自检 开始");
    while(Inf_Si24R1_Check() == 1)
    {
        HAL_Delay(500);
    }
    debug_printfln(" 2.4g模块自检 结束");

    debug_printfln(" 2.4g设为接收模式");
    Inf_Si24R1_RXMode();

    debug_printfln("通讯模块初始化 结束");
}

/**
 * @description: 解析摇杆数据
 * @return {*} Com_Status 是否接收到: Com_OK接收到 否则没有接收到
 */
Com_Status App_Communication_ReceiveJoyStickData(void)
{
    /* 1. 读取数据, 如果没有接收到, 直接返回 fail */
    taskENTER_CRITICAL();
    uint8_t r = Inf_Si24R1_RxPacket(RX_BUFF);
    taskEXIT_CRITICAL();
    if(r == 1) return Com_FAIL;

    /* 2. 接收到数据: 判断帧头是否匹配(是否是发给自己的遥控器数据) */
    if(RX_BUFF[0] != FRAME_0 ||
       RX_BUFF[1] != FRAME_1 ||
       RX_BUFF[2] != FRAME_2) return Com_FAIL;

    /* 3. 校验和验证: 防止数据在无线传输中损坏 */
    uint8_t payloadLen = RX_BUFF[3];
    uint8_t totalLen   = 3 + 1 + payloadLen + 4; /* 帧头3 + 长度1 + 数据N + 校验4 */
    if (totalLen > RX_PLOAD_WIDTH) return Com_FAIL;
    if (!App_Communication_VerifyChecksum(RX_BUFF, totalLen)) return Com_FAIL;

    /* 4. 解析数据负载 */
    joyStick.THR = (RX_BUFF[4] << 8) | RX_BUFF[5];
    joyStick.YAW = (RX_BUFF[6] << 8) | RX_BUFF[7];
    joyStick.PIT = (RX_BUFF[8] << 8) | RX_BUFF[9];
    joyStick.ROL = (RX_BUFF[10] << 8) | RX_BUFF[11];

    joyStick.isPowerDown = RX_BUFF[12];

    if(RX_BUFF[13]) /* 当 RX_BUFF[13] 为1时  切换定高模式 */
    {
        joyStick.isFixHeight = !joyStick.isFixHeight;

        isFixHeight = joyStick.isFixHeight == 1 ? Com_OK : Com_FAIL;
    }

    return Com_OK;
}

/**
 * @description: 判断和遥控器是否连接
 *  连接成功:
 *      只要收到一个数据,就算连接成功
 *  失败:
 *      如果连续 1.2s 没有收到数据,表示失败
 * @param {Com_Status} isReceiveData 是否收到遥控器数据
 * @return {*}  是否连接成功
 */
Com_Status App_Communication_CheckConnection(Com_Status isReceiveData)
{                             /* 记录连续未收到数据的次数 */
    static uint8_t cnt = FAILSAFE_TIMEOUT_CNT; /* 初始化为200,以防止一开始的时候,没有连接成功表示连接成功...*/
    /* 1. 如果收到数据,表示连接成功 */
    if(isReceiveData == Com_OK)
    {
        cnt = 0;
        return Com_OK;
    }

    /* 2. 表示没有收到数据: 计时, 超过200次(1.2s) */
    cnt++;
    if(cnt >= FAILSAFE_TIMEOUT_CNT)
    {
        cnt = FAILSAFE_TIMEOUT_CNT; /* 避免溢出 */
        return Com_FAIL;
    }

    return Com_OK;
}

/**
 * @description: 遥控器的解锁
 *  油门拉到最大值,保持1s,然后拉到最小值,保持1s, 解锁成功.
 *
 *   自由状态
 *       在此状态, 如果油门拉满(>=960), 进入  拉到最大值 状态
 *   拉到最大值
 *       如果油门拉满持续时间超过1s ,离开最大值状态
 *       如果油门拉满时间不到1s, 退回到 自由状态
 *   离开最大值
 *       当油门摇杆 <=20, 进入最小值状态
 *   拉到最小值
 *       如果持续的时间超过1s, 进入解锁状态
 *       如果持续时间不到1s, 回到自由状态
 *   解锁
 *
 *
 *
 *
 * @param {Com_Status} isRemoteConnected 是否连接成功
 * @return {*} 是否解锁成功 Com_OK 解锁成功  否则 解锁失败
 */
Com_Status App_Communication_RemoteUnlock(Com_Status isRemoteConnected)
{
    /* 拉最大值和最小值的持续时间 */
    static uint8_t thrMaxDuration = 0;
    static uint8_t thrMinDuration = 0;
    /* 油门的解锁状态 */
    static Com_RemoteStatus remoteStatus = THR_FREE;

    /* 1. 如果是失败状态, 直接解锁失败 */
    if(isRemoteConnected != Com_OK)
    {
        remoteStatus         = THR_FREE;
        joyStick.isFixHeight = 0;
        isFixHeight          = Com_FAIL;
        return Com_FAIL;
    }

    switch(remoteStatus)
    {
        case THR_FREE:
        {
            /* 在自由状态, 最大值和最小值持续时间都归零 */
            thrMaxDuration = 0;
            thrMinDuration = 0;
            if(joyStick.THR >= ARM_THR_HIGH)
            {
                remoteStatus = THR_MAX;
            }
            break;
        }
        case THR_MAX:
        {
            if(joyStick.THR >= ARM_THR_HIGH)
            {
                /* 判断持续时间 */
                thrMaxDuration++;
                if(thrMaxDuration >= ARM_DURATION_CNT)
                {
                    remoteStatus = THR_MAX_LEAVE;
                }
            }
            else
            {
                /* 证明拉最大值的持续时间没有加够1.2s */
                remoteStatus = THR_FREE; /* 回到自由状态 */
            }
            break;
        }
        case THR_MAX_LEAVE:
        {
            if(joyStick.THR <= ARM_THR_LOW)
            {
                remoteStatus = THR_MIN;
            }
            break;
        }
        case THR_MIN:
        {
            if(joyStick.THR <= ARM_THR_LOW)
            {
                thrMinDuration++;
                if(thrMinDuration >= ARM_DURATION_CNT)
                {
                    remoteStatus = THR_UNLOCK;
                }
            }
            else
            {
                remoteStatus = THR_FREE;
            }
            break;
        }
        case THR_UNLOCK:
        {
            /* 在解锁状态里,油门持续时间(1min) <=20, 则上锁 */
            static uint32_t lowDuration = 0;
            if(joyStick.THR <= ARM_THR_LOW)
            {
                lowDuration++;
                if(lowDuration >= DISARM_DURATION_CNT)
                {
                    remoteStatus = THR_FREE;
                    lowDuration  = 0;
                }
            }
            else
            {
                lowDuration = 0;
            }

            /* 解锁成功, 直接返回成功 */
            return Com_OK;
        }

        default:
            break;
    }

    joyStick.isFixHeight = 0;
    isFixHeight          = Com_FAIL;
    /* 默认返回解锁失败 */
    return Com_FAIL;
}
