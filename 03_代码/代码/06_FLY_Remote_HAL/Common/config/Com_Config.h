#ifndef __COM_CONFIG_H
#define __COM_CONFIG_H

#include "stdint.h"
#include "stdio.h"
#include "Com_RemoteConfig.h"

typedef enum
{
    Com_OK = 0,
    Com_TIMEOUT,
    Com_FAIL
} Com_Status;

#define LIMIT(x, min, max) (x) >= (max) ? (max) : ((x) <= (min) ? (min) : (x))

typedef enum
{
    KEY_NONE = 0,
    KEY_LEFT,
    KEY_UP,
    KEY_DOWN,
    KEY_RIGHT,
    KEY_LEFT_TOP,
    KEY_RIGHT_TOP,
    KEY_LEFT_TOP_LONG,
    KEY_RIGHT_TOP_LONG
} Com_Key;

typedef struct
{
    int16_t THR; /* ���� */
    int16_t PIT; /* ���� */
    int16_t ROL; /* ��� */
    int16_t YAW; /* ƫ�� */

    uint8_t isPowerDown; /* �Ƿ�ػ�: 1:�ػ� 0:���ػ� */
    uint8_t isFixHeight; /* �Ƿ�ת���ߵ�״̬ */
} JoyStick_Struct;

extern JoyStick_Struct joyStick;
extern JoyStick_Struct joyStickBias;

void Com_Config_PrintJoyStick(uint8_t *pre);
void Com_Config_PrintJoyStickBias(uint8_t *pre);
#endif
