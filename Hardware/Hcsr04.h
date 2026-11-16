#ifndef __HCSR04_H
#define __HCSR04_H

#include "stm32f10x.h"

void Hcsr04_Init(void);
void Hcsr04_StartMeasure(void);
void Hcsr04_Reset(void);
float Hcsr04_GetDistance_cm(void);

#endif
