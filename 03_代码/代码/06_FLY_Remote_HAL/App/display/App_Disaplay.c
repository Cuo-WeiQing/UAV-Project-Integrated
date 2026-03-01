#include "App_Disaplay.h"

void App_Display_ShowProgressBar(uint8_t temp,
                                 uint8_t chr_star,
                                 uint8_t chr_default,
                                 uint8_t x,
                                 uint8_t y,
                                 uint8_t size,
                                 uint8_t mode);
void App_Display_Start(void)
{
    OLED_Init();
}
void App_Display_Show(void)
{
    uint8_t temp;
    ///////////////////////////////第一行///////////////////////////////////
    OLED_Show_CH(Line1_Begin + 00, Y0, 0, 12, 1);
    OLED_Show_CH(Line1_Begin + 12, Y0, 1, 12, 1);
    OLED_Show_CH(Line1_Begin + 24, Y0, 2, 12, 1);
    OLED_Show_CH(Line1_Begin + 36, Y0, 3, 12, 1);
    OLED_Show_CH(Line1_Begin + 48, Y0, 4, 12, 1);
    OLED_Show_CH(Line1_Begin + 60, Y0, 5, 12, 1);

    OLED_ShowNumber(2, Y0, CH, 3, 12);   // 显示无线信道
    ///////////////////////////////第二行///////////////////////////////////

    ///////////////////////////////第三、四行/////////////////////////////////
    // 显示遥控数据
    OLED_ShowString(Line3_Begin + 00, Y2, "THR:", 12, 1);
    temp = joyStick.THR / 41;
    App_Display_ShowProgressBar(temp, 12, 24, Line4_Begin + 6, Y2, 12, 1);
    temp = (joyStick.THR - 500) / 41;
    App_Display_ShowProgressBar(temp, 12, 12, Line4_Begin + 18, Y2, 12, 1);

    OLED_ShowString(Line3_Begin + 64, Y2, "ROL:", 12, 1);
    temp = joyStick.ROL / 41;
    App_Display_ShowProgressBar(temp, 12, 24, Line4_Begin + 70, Y2, 12, 1);
    temp = (joyStick.ROL - 500) / 41;
    App_Display_ShowProgressBar(temp, 12, 12, Line4_Begin + 82, Y2, 12, 1);

    OLED_ShowString(Line3_Begin + 00, Y3, "YAW:", 12, 1);
    temp = joyStick.YAW / 41;
    App_Display_ShowProgressBar(temp, 12, 24, Line4_Begin + 6, Y3, 12, 1);
    temp = (joyStick.YAW - 500) / 41;
    App_Display_ShowProgressBar(temp, 12, 12, Line4_Begin + 18, Y3, 12, 1);

    OLED_ShowString(Line3_Begin + 64, Y3, "PIT:", 12, 1);
    temp = joyStick.PIT / 41;
    App_Display_ShowProgressBar(temp, 12, 24, Line4_Begin + 70, Y3, 12, 1);
    temp = (joyStick.PIT - 500) / 41;
    App_Display_ShowProgressBar(temp, 12, 12, Line4_Begin + 82, Y3, 12, 1);

    OLED_Refresh_Gram();   // 开始显示
}

// 进度条显示函数
void App_Display_ShowProgressBar(uint8_t temp,
                                 uint8_t chr_star,
                                 uint8_t chr_default,
                                 uint8_t x,
                                 uint8_t y,
                                 uint8_t size,
                                 uint8_t mode)
{
    switch(temp)
    {
        case 0: OLED_Show_CH(x, y, chr_star + temp, size, size); break;
        case 1: OLED_Show_CH(x, y, chr_star + temp, size, size); break;
        case 2: OLED_Show_CH(x, y, chr_star + temp, size, size); break;
        case 3: OLED_Show_CH(x, y, chr_star + temp, size, size); break;
        case 4: OLED_Show_CH(x, y, chr_star + temp, size, size); break;
        case 5: OLED_Show_CH(x, y, chr_star + temp, size, size); break;
        case 6: OLED_Show_CH(x, y, chr_star + temp, size, size); break;
        case 7: OLED_Show_CH(x, y, chr_star + temp, size, size); break;
        case 8: OLED_Show_CH(x, y, chr_star + temp, size, size); break;
        case 9: OLED_Show_CH(x, y, chr_star + temp, size, size); break;
        case 10: OLED_Show_CH(x, y, chr_star + temp, size, size); break;
        case 11: OLED_Show_CH(x, y, chr_star + temp, size, size); break;
        case 12: OLED_Show_CH(x, y, chr_star + temp, size, size); break;

        default: OLED_Show_CH(x, y, chr_default, size, size); break;
    }
}
