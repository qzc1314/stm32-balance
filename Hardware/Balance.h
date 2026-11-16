#ifndef BALANCE_H
#define BALANCE_H

#include "stm32f10x.h"

/* ===== 三个必须实测的数 ===== */
#define BALANCE_ZERO_POINT     0.0f
        /* 静置热机 2 分钟后读 PIT。绝对不要填 1.884 */
#define PITCH_SIGN             (-1.0f)
        /* 实测：按当前 IMU 安装方向，车头（超声波侧）向下倾时
         * attitude.pitch 为负，而控制需要"前倾为正"（前倾→向前加速）。
         * 取 -1 后 PIT 显示也是前倾为正。
         * 如果换成 +1，车会朝倾倒方向加速（正反馈），越倒越快。 */
#define MOTOR_DEADZONE         400.0f
        /* 起点值，必须跑阶梯测试后替换。范围大概 600~1400 */

#define FALL_ANGLE_THRESHOLD   45.0f
#define SPEED_TARGET_MAX       500.0f   /* target_speed 上限，脉冲/10ms */

extern volatile float target_speed;
extern volatile float target_turn;

extern volatile int16_t dbg_speed_l;
extern volatile int16_t dbg_speed_r;
extern volatile float   dbg_speed_target;
extern volatile float   dbg_speed_actual;
extern volatile float   dbg_pitch;
extern volatile float   dbg_u;

extern volatile uint8_t system_fault;

void Balance_Init(void);
void Balance_Calc(float dt);

#endif

