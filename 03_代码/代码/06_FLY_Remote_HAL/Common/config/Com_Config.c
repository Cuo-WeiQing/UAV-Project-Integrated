#include "Com_Config.h"

/* 摇杆数据结构体 */
JoyStick_Struct joyStick;

/* 存储摇杆偏移量的结构体 */
JoyStick_Struct joyStickBias;

void Com_Config_PrintJoyStick(uint8_t *pre)
{
    printf("%s thr = %d, pit = %d, rol = %d, yaw = %d\r\n",
           pre,
           joyStick.THR,
           joyStick.PIT,
           joyStick.ROL,
           joyStick.YAW);
}

void Com_Config_PrintJoyStickBias(uint8_t *pre)
{
    printf("%s thr = %d, pit = %d, rol = %d, yaw = %d\r\n",
           pre,
           joyStickBias.THR,
           joyStickBias.PIT,
           joyStickBias.ROL,
           joyStickBias.YAW);
}

