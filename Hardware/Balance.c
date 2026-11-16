#include "balance.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "pid.h"
#include <math.h>

volatile float target_speed = 0.0f;
volatile float target_turn  = 0.0f;

volatile int16_t dbg_speed_l = 0;
volatile int16_t dbg_speed_r = 0;
volatile float   dbg_speed_target = 0.0f;
volatile float   dbg_speed_actual = 0.0f;
volatile float   dbg_pitch = 0.0f;
volatile float   dbg_u = 0.0f;

volatile uint8_t system_fault = 0;

static PID_t pid_speed;
static PID_t pid_turn;

/* ===== 直立环 PD（并联结构，输出为 PWM） ===== */
#define BALANCE_KP        80.0f
#define BALANCE_KD        1.5f

/* ===== 速度环 PI（输出直接叠加到 PWM，靠限幅抗饱和） ===== */
#define SPEED_KP          15.0f
#define SPEED_KI          0.8f
#define SPEED_U_MAX       2000.0f          /* 速度环最大权限 ≈28% 占空比 */
#define SPEED_I_MAX       (800.0f / SPEED_KI)

#define TURN_KP           20.0f
#define TURN_MAX_OUT      2000.0f

/* 输出迟滞门限 */
#define OUT_ON            60.0f
#define OUT_OFF           25.0f

static int8_t out_dir_l = 0;
static int8_t out_dir_r = 0;

static void Motor_Out(uint8_t left, float u)
{
    int8_t *state = left ? &out_dir_l : &out_dir_r;
    float mag = fabsf(u);
    int8_t dir;

    if (mag > (float)MOTOR_PWM_MAX) mag = (float)MOTOR_PWM_MAX;

    /* 换向前先把占空比清零，避免 H 桥带载反向 */
    if (*state != 0 && ((u > 0.0f) != (*state > 0))) {
        if (left) MotorA_SetDuty(0);
        else      MotorB_SetDuty(0);
        *state = 0;
    }

    if (*state == 0) {
        if (mag < OUT_ON) {
            if (left) { MotorA_SetDir(MOTOR_STOP); MotorA_SetDuty(0); }
            else      { MotorB_SetDir(MOTOR_STOP); MotorB_SetDuty(0); }
            return;
        }
    } else {
        if (mag < OUT_OFF) {
            if (left) { MotorA_SetDir(MOTOR_STOP); MotorA_SetDuty(0); }
            else      { MotorB_SetDir(MOTOR_STOP); MotorB_SetDuty(0); }
            *state = 0;
            return;
        }
    }

    dir = (u > 0.0f) ? 1 : -1;
    mag += MOTOR_DEADZONE;
    if (mag > (float)MOTOR_PWM_MAX) mag = (float)MOTOR_PWM_MAX;

    if (left) {
        MotorA_SetDir(dir > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD);
        MotorA_SetDuty((uint16_t)mag);
    } else {
        MotorB_SetDir(dir > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD);
        MotorB_SetDuty((uint16_t)mag);
    }
    *state = dir;
}

void Balance_Init(void)
{
    PID_Init(&pid_speed, SPEED_KP, SPEED_KI, 0.0f,
             SPEED_U_MAX, SPEED_I_MAX, 0.0f, 0.0f, 1);
    /* 转向环方向取 -1：实测原方向是正反馈，车会自己越转越快。
     * 取反后陀螺仪 Z 轴角速度成为阻尼，车才会"抵抗"被转。 */
    PID_Init(&pid_turn, TURN_KP, 0.0f, 0.0f,
             TURN_MAX_OUT, 0.0f, 0.0f, 0.0f, -1);
}

void Balance_Calc(float dt)
{
    if (!(dt > 0.001f)) dt = 0.01f;

    if (system_fault) { Motor_AllStop(); return; }

    float pitch = PITCH_SIGN * attitude.pitch;
    float gyro  = PITCH_SIGN * attitude.gyro_x;

    dbg_pitch = pitch;

    if (fabsf(pitch - BALANCE_ZERO_POINT) > FALL_ANGLE_THRESHOLD) {
        system_fault = 1;
        Motor_AllStop();
        return;
    }

    /* 编码器增量由 Encoder_Update 在节拍里已经取好，这里只读缓存 */
    int16_t dl = EncoderA_GetDelta();
    int16_t dr = EncoderB_GetDelta();

    /* 换算成“脉冲/10ms”，dt 偏离 10ms 时保持量纲 */
    float speed_avg = 0.5f * (float)((int32_t)dl + (int32_t)dr) * (0.01f / dt);

    /* 目标限幅：蓝牙/上位机可能给出超出机械能力的指令 */
    float sp = target_speed;
    float tn = target_turn;
    if (sp >  SPEED_TARGET_MAX) sp =  SPEED_TARGET_MAX;
    if (sp < -SPEED_TARGET_MAX) sp = -SPEED_TARGET_MAX;

    /* 直立环：前倾 -> pitch 大 -> u 正 -> 前进 */
    float u_pd = BALANCE_KP * (pitch - BALANCE_ZERO_POINT)
               + BALANCE_KD * gyro;

    /* 速度环：输出直接叠加，限幅保证不会盖过直立环 */
    float u_speed = PID_Calc(&pid_speed, sp, speed_avg, dt);
    if (u_speed >  SPEED_U_MAX) u_speed =  SPEED_U_MAX;
    if (u_speed < -SPEED_U_MAX) u_speed = -SPEED_U_MAX;

    float u = u_pd + u_speed;

    /* 转向环：以车体角速度为反馈，target_turn 单位是 °/s */
    float turn = PID_Calc(&pid_turn, tn, attitude.gyro_z, dt);

    float pwm_left  = u + turn;
    float pwm_right = u - turn;

    float max_mag = fabsf(pwm_left);
    if (fabsf(pwm_right) > max_mag) max_mag = fabsf(pwm_right);
    if (max_mag > (float)MOTOR_PWM_MAX) {
        float scale = (float)MOTOR_PWM_MAX / max_mag;
        pwm_left  *= scale;
        pwm_right *= scale;
    }

    dbg_speed_l = dl;
    dbg_speed_r = dr;
    dbg_speed_target = sp;
    dbg_speed_actual = speed_avg;
    dbg_u = u;

    Motor_Out(1, pwm_left);
    Motor_Out(0, pwm_right);
}


