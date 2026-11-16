#ifndef PID_H
#define PID_H

#include "stm32f10x.h"

typedef struct {
    float target;
    float actual;
    float Kp, Ki, Kd;
    float err, last_err;
    float last_actual;
    float integral;
    float max_out, max_i;
    float dead_zone;
    float d_alpha;      /* 微分低通：越接近 1 滤波越强，τ≈dt/(1-α)；直立环 0.4~0.6，速度环 Kd=0 */
    float last_d;
    int8_t direction;   /* +1 正作用，-1 反作用 */
    float output;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float max_out, float max_i,
              float dead_zone, float d_alpha, int8_t direction);

void PID_Seed(PID_t *pid, float actual);
void PID_Reset(PID_t *pid);

float PID_Calc(PID_t *pid, float target, float actual, float dt);

#endif
