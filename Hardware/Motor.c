#include "stm32f10x.h"                  // Device header

#include "motor.h"

void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_TIM1, ENABLE);

    // PA8(TIM1_CH1)、PA11(TIM1_CH4) 复用推挽输出
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 方向控制引脚 PB12/13/14/15
    GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

    // 定时器基配置：72MHz / 7200 = 10kHz
    TIM_TimeBaseStruct.TIM_Prescaler         = 0;
    TIM_TimeBaseStruct.TIM_Period            = 7199;
    TIM_TimeBaseStruct.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    // CH1 → B路电机
    TIM_OCInitStruct.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStruct.TIM_Pulse        = 0;
    TIM_OCInitStruct.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM1, &TIM_OCInitStruct);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // CH4 → A路电机
    TIM_OCInitStruct.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStruct.TIM_Pulse        = 0;
    TIM_OCInitStruct.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OC4Init(TIM1, &TIM_OCInitStruct);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    // 启动定时器 + 主输出使能
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void MotorA_SetDuty(uint16_t duty)
{
    if(duty > MOTOR_PWM_MAX) duty = MOTOR_PWM_MAX;
    TIM_SetCompare4(TIM1, duty);
}

void MotorA_SetDir(MotorDir_t dir)
{
    switch(dir)
    {
        case MOTOR_FORWARD:
            GPIO_SetBits(GPIOB, GPIO_Pin_14);
            GPIO_ResetBits(GPIOB, GPIO_Pin_15);
            break;
        case MOTOR_BACKWARD:
            GPIO_ResetBits(GPIOB, GPIO_Pin_14);
            GPIO_SetBits(GPIOB, GPIO_Pin_15);
            break;
        case MOTOR_BRAKE:
            MotorA_SetDuty(0);   // 关闭PWM，两脚同高实现刹车
            GPIO_SetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15);
            break;
        default: // MOTOR_STOP
            MotorA_SetDuty(0);
            GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15);
            break;
    }
}

void MotorB_SetDuty(uint16_t duty)
{
    if(duty > MOTOR_PWM_MAX) duty = MOTOR_PWM_MAX;
    TIM_SetCompare1(TIM1, duty);
}

void MotorB_SetDir(MotorDir_t dir)
{
    switch(dir)
    {
        case MOTOR_FORWARD:
            GPIO_SetBits(GPIOB, GPIO_Pin_12);
            GPIO_ResetBits(GPIOB, GPIO_Pin_13);
            break;
        case MOTOR_BACKWARD:
            GPIO_ResetBits(GPIOB, GPIO_Pin_12);
            GPIO_SetBits(GPIOB, GPIO_Pin_13);
            break;
        case MOTOR_BRAKE:
            MotorB_SetDuty(0);
            GPIO_SetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13);
            break;
        default: // MOTOR_STOP
            MotorB_SetDuty(0);
            GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13);
            break;
    }
}

void Motor_AllStop(void)
{
    MotorA_SetDuty(0);
    MotorB_SetDuty(0);
    MotorA_SetDir(MOTOR_STOP);
    MotorB_SetDir(MOTOR_STOP);
}


