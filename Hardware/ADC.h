#ifndef ADC_H
#define ADC_H

#include "stm32f10x.h"

void Adc_Init(void);
float Get_Battery_Voltage(void);

#endif
