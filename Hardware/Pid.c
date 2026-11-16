#include "stm32f10x.h"
#include "pid.h"
#include <math.h>

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float max_out, float max_i,
              float dead_zone, float d_alpha, int8_t direction)
{
    pid->target = 0;
    pid->actual = 0;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->err = 0;
    pid->last_err = 0;
    pid->last_actual = 0;
    pid->integral = 0;
    pid->max_out = max_out;
    pid->max_i = max_i;
    pid->dead_zone = dead_zone;
    pid->d_alpha = d_alpha;
    pid->last_d = 0;
    pid->direction = direction;
    pid->output = 0;
}

void PID_Seed(PID_t *pid, float actual)   /* 初始化时喂真实值 */
{
    pid->actual = actual;
    pid->last_actual = actual;
    pid->last_d = 0.0f;
}

void PID_Reset(PID_t *pid)
{
    pid->err = 0;
    pid->last_err = 0;
    pid->integral = 0;
    pid->last_d = 0;
    pid->output = 0;
}

float PID_Calc(PID_t *pid, float target, float actual, float dt)
{
    if (dt < 1e-6f) dt = 0.01f;
    if (dt > 0.05f) dt = 0.01f;          /* 周期异常兜底 */

    pid->target = target;
    pid->actual = actual;
    pid->err = target - actual;

    float err_eff = pid->err;
    if (fabsf(err_eff) < pid->dead_zone) err_eff = 0.0f;

    float p_out = pid->Kp * err_eff;


    float d_out = 0.0f;
    if (pid->Kd != 0.0f) {
        float d_raw = -(actual - pid->last_actual) / dt;
        pid->last_d = pid->d_alpha * pid->last_d
                    + (1.0f - pid->d_alpha) * d_raw;
        d_out = pid->Kd * pid->last_d;
    }

    float integral_new = pid->integral + err_eff * dt;
    if (integral_new >  pid->max_i) integral_new =  pid->max_i;
    if (integral_new < -pid->max_i) integral_new = -pid->max_i;

    float out = p_out + pid->Ki * integral_new + d_out;
    out *= (float)pid->direction;

    if (out > pid->max_out) {
        out = pid->max_out;
        if (err_eff * pid->direction > 0) integral_new = pid->integral;
    } else if (out < -pid->max_out) {
        out = -pid->max_out;
        if (err_eff * pid->direction < 0) integral_new = pid->integral;
    }

    pid->integral = integral_new;
    pid->last_err = pid->err;
    pid->last_actual = actual;
    pid->output = out;
    return out;
}

