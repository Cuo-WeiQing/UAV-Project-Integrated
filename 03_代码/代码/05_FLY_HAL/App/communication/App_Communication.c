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

    debug_printfln(" 2.4设置为接收模式");
    Inf_Si24R1_RXMode();

    debug_printfln("通讯模块的启动 结束");
}

/**
 * @description: 接收摇杆数据
 * @return {*} Com_Status 是否收到数据: Com_OK收到 否则没有收到
 */
Com_Status App_Communication_ReceiveJoyStickData(void)
{
    /* 1. 接收数据, 如果没有收到, 直接返回 fail */
    taskENTER_CRITICAL();
    uint8_t r = Inf_Si24R1_RxPacket(RX_BUFF);
    taskEXIT_CRITICAL();
    if(r == 1) return Com_FAIL;

    /* 2. 收到数据: 判断帧头是否相等(是否是自己的遥控器发的) */
    if(RX_BUFF[0] != FRAME_0 ||
       RX_BUFF[1] != FRAME_1 ||
       RX_BUFF[2] != FRAME_2) return Com_FAIL;

    /* 3. 真正的数据 */
    joyStick.THR = (RX_BUFF[4] << 8) | RX_BUFF[5];
    joyStick.YAW = (RX_BUFF[6] << 8) | RX_BUFF[7];
    joyStick.PIT = (RX_BUFF[8] << 8) | RX_BUFF[9];
    joyStick.ROL = (RX_BUFF[10] << 8) | RX_BUFF[11];

    joyStick.isPowerDown = RX_BUFF[12];

    if(RX_BUFF[13]) /* 当 RX_BUFF[13] 是1时  切换定高模式 */
    {
        joyStick.isFixHeight = !joyStick.isFixHeight;

        isFixHeight = joyStick.isFixHeight == 1 ? Com_OK : Com_FAIL;
    }

    return Com_OK;
}

/**
 * @description: 用来检测遥控器的连接情况
 *  连接成功:
 *      只要收到一条数据,就算连接成功
 *  失联:
 *      如果连续 1.2s 没有收到数据,表示失联
 * @param {Com_Status} isReceiveData 是否收到遥控器数据
 * @return {*}  是否连接成功
 */
Com_Status App_Communication_CheckConnection(Com_Status isReceiveData)
{                             /* 记录连续收不到数据的次数 */
    static uint8_t cnt = 200; /* 初始化为200可以防止一启动时候,没有连接成功显示连接成功问题*/
    /* 1. 如果收到数据,表示连接成功 */
    if(isReceiveData == Com_OK)
    {
        cnt = 0;
        return Com_OK;
    }

    /* 2. 表示没有收到数据: 计时, 连续200次(1.2s) */
    cnt++;
    if(cnt >= 200)
    {
        cnt = 200; /* 避免溢出 */
        return Com_FAIL;
    }

    return Com_OK;
}

/**
 * @description: 遥控器的解锁
 *  把油门拉到最高,持续1s,然后再拉到最低,持续1s, 解锁成功.
 *
 *   自由状态
 *       在此状态, 如果油门拉到最高(>=960), 进入  最大值附近 状态
 *   最大值附近
 *       如果油门持续时间超过1s ,离开最大值状态
 *       如果在这里持续时间不到1s, 退回到 自由状态
 *   离开最大值
 *       如果出现油门 <=20, 进入最小值附近
 *   最小值附近
 *       如果持续时间大于1s, 进入解锁状态
 *       如果持续时间不到1s, 进入自由状态
 *   解锁
 *
 *
 *
 *
 * @param {Com_Status} isRemoteConnected 是否连接成功
 * @return {*} 是否解锁成功 Com_OK 解锁成功  其他 解锁失败
 */
Com_Status App_Communication_RemoteUnlock(Com_Status isRemoteConnected)
{
    /* 油门在最大值和最小值附近的持续时间 */
    static uint8_t thrMaxDuration = 0;
    static uint8_t thrMinDuration = 0;
    /* 定义油门的状态 */
    static Com_RemoteStatus remoteStatus = THR_FREE;

    /* 1. 如果失联状态, 直接解锁失败 */
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
            /* 在自由状态, 最大值和最小值附近持续时间从0计数 */
            thrMaxDuration = 0;
            thrMinDuration = 0;
            if(joyStick.THR >= 960)
            {
                remoteStatus = THR_MAX;
            }
            break;
        }
        case THR_MAX:
        {
            if(joyStick.THR >= 960)
            {
                /* 判断持续时间 */
                thrMaxDuration++;
                if(thrMaxDuration >= 200)
                {
                    remoteStatus = THR_MAX_LEAVE;
                }
            }
            else
            {
                /* 证明在最大值附近没有坚持1.2s */
                remoteStatus = THR_FREE; /* 回到自由状态 */
            }
            break;
        }
        case THR_MAX_LEAVE:
        {
            if(joyStick.THR <= 20)
            {
                remoteStatus = THR_MIN;
            }
            break;
        }
        case THR_MIN:
        {
            if(joyStick.THR <= 20)
            {
                thrMinDuration++;
                if(thrMinDuration >= 200)
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
            /* 如果解锁状态下,油门长时间(1min) <=20, 马上锁定 */
            static uint32_t lowDuration = 0;
            if(joyStick.THR <= 20)
            {
                lowDuration++;
                if(lowDuration >= 200 * 50)
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
