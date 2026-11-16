#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f10x.h"

#define ENCODER_LINE        13      // 编码器线数，按实际电机修改
#define REDUCTION_RATIO     90      // 减速比
#define PULSE_PER_ROUND     (ENCODER_LINE * REDUCTION_RATIO * 4)
#define SAMPLE_PERIOD_MS    10      // 采样周期 10ms

void Encoder_Init(void);
void Encoder_Update(void);
int16_t EncoderA_GetDelta(void);
int16_t EncoderB_GetDelta(void);
float EncoderA_GetSpeed_RPM(void);
float EncoderB_GetSpeed_RPM(void);

#endif
