#include "Inf_Motor.h"
/**
 * @description: 电机的初始化
 * @return {*}
 */
void Inf_Motor_Init(void)
{
    /* 1. 启动4个定时器 */
    /*  left-top tim3-c1 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    /* 2. 4个motor工作起来 */
    motorLeftTop.speed    = 0;
    motorLeftBottom.speed = 0;

    motorRightTop.speed    = 0;
    motorRightBottom.speed = 0;
    Inf_Motor_AllMotorsWork();
}

/**
 * @description: 设置指定的电机的速度
 * @param {Motor_Struct} *motor
 * @return {*}
 */
void Inf_Motor_SetSpeed(Motor_Struct *motor)
{
    /* 对motor的速度做一个限幅 */
    motor->speed = LIMIT(motor->speed, 0, 1000);

    if(motor->location == LEFT_TOP)
    {
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, motor->speed);
    }
    else if(motor->location == LEFT_BOTTOM)
    {
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, motor->speed);
    }
    else if(motor->location == RIGHT_TOP)
    {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, motor->speed);
    }
    else if(motor->location == RIGHT_BOOTTOM)
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, motor->speed);
    }
}

void Inf_Motor_AllMotorsWork(void)
{
    Inf_Motor_SetSpeed(&motorLeftTop);
    Inf_Motor_SetSpeed(&motorLeftBottom);
    Inf_Motor_SetSpeed(&motorRightTop);
    Inf_Motor_SetSpeed(&motorRightBottom);
}
