#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f10x.h"

typedef struct {
    float pitch;
    float gyro_x;
    float gyro_z;
} Attitude_t;

extern Attitude_t attitude;

void MPU6050_Init(void);
uint8_t MPU6050_IsReady(void);
uint8_t MPU6050_GetId(void);
void MPU6050_GetData(float dt);

#endif
