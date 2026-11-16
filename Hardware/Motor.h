#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f10x.h"

#define MOTOR_PWM_MAX    7199   // 与 ARR 一致，对应 100% 占空比

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_BRAKE
} MotorDir_t;

void Motor_Init(void);
void MotorA_SetDuty(uint16_t duty);
void MotorA_SetDir(MotorDir_t dir);
void MotorB_SetDuty(uint16_t duty);
void MotorB_SetDir(MotorDir_t dir);
void Motor_AllStop(void);

#endif
