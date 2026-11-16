#include "stm32f10x.h"
#include "motor.h"
#include "encoder.h"
#include "bluetooth.h"
#include "hcsr04.h"
#include "adc.h"
#include "mpu6050.h"
#include "balance.h"
#include "oled.h"
#include "delay.h"

volatile uint32_t systick_cnt = 0;

/* ===== 节拍 ===== */
#define CTRL_PERIOD_MS       10      /* 控制周期 */
#define OLED_PERIOD_MS       200     /* 显示刷新（只刷显存） */
#define VOLT_PERIOD_MS       1000    /* 电池检测 */

/* ===== 障碍物 ===== */
#define ENABLE_SONAR         0       /* 触发时序已修正，测通后可改 1 */
#define SONAR_PERIOD_MS      60
#define SONAR_WAIT_MS        35      /* 4m 回波约 23ms，留出余量 */
#define OBSTACLE_STOP_CM     20.0f
#define OBSTACLE_CLEAR_CM    30.0f

/* ===== 电池 ===== */
#define LOW_VOLTAGE_THRESHOLD  6.5f
#define LOW_VOLTAGE_SAMPLES    3     /* 连续 3 次低压才认定故障 */

/* ===== 看门狗 ===== */
#define USE_WATCHDOG         1       /* 调试时不想被复位可改 0 */

/* ===== 蓝牙失联保护 ===== */
#define BT_CMD_TIMEOUT_MS    500     /* 超过 500ms 没收到命令就停车 */

static uint8_t obstacle_flag = 0;
static uint8_t low_volt_cnt  = 0;
static uint8_t low_volt_latched = 0;
static uint32_t bt_last_cmd_ms = 0;   /* 最后一次收到蓝牙命令的时刻 */
static uint8_t  bt_link_active = 0;   /* 是否处于蓝牙控制中 */

#if USE_WATCHDOG
/* IWDG：LSI 约 40kHz，64 分频后 625Hz，重装 250 约 400ms */
static void Watchdog_Init(void)
{
    IWDG->KR  = 0xCCCC;      /* 启动看门狗 */
    IWDG->KR  = 0x5555;      /* 允许写 PR/RLR */
    IWDG->PR  = 0x04;        /* 64 分频 */
    IWDG->RLR = 250;
    while (IWDG->SR);        /* 等待寄存器更新 */
    IWDG->KR  = 0xAAAA;      /* 喂狗 */
}

static void Watchdog_Feed(void)
{
    IWDG->KR = 0xAAAA;
}
#endif

/* 刷新一帧显存（真正刷屏在 10ms 拍里分页完成） */
static void Display_Update(void)
{
    float v = Get_Battery_Voltage();
    uint16_t v10;

    if (v < 0.0f) v = 0.0f;
    v10 = (uint16_t)(v * 10.0f + 0.5f);

    OLED_Clear();

    OLED_ShowString(1, 1, "Pit:");
    OLED_ShowSignedNum(1, 5, (int32_t)dbg_pitch, 3);

    OLED_ShowString(2, 1, "L:");
    OLED_ShowSignedNum(2, 3, dbg_speed_l, 4);
    OLED_ShowString(2, 9, "R:");
    OLED_ShowSignedNum(2, 11, dbg_speed_r, 4);

    OLED_ShowString(3, 1, "Bat:");
    OLED_ShowNum(3, 5, v10 / 10, 1);
    OLED_ShowChar(3, 6, '.');
    OLED_ShowNum(3, 7, v10 % 10, 1);
    OLED_ShowString(3, 8, "V");

    OLED_ShowString(4, 1, "S:");
    OLED_ShowSignedNum(4, 3, (int32_t)dbg_speed_target, 3);
    OLED_ShowString(4, 8, system_fault ? "ERR" : "OK ");
}

int main(void)
{
    Motor_Init();
    Encoder_Init();

    SysTick_Config(SystemCoreClock / 1000);
    NVIC_SetPriority(SysTick_IRQn, 0);

    Delay_ms(50);

    MPU6050_Init();
    Balance_Init();
    Bluetooth_Init();
    Adc_Init();
    Hcsr04_Init();
    OLED_Init();

    /* 传感器不在线时直接进入故障态，禁止电机输出 */
    if (!MPU6050_IsReady()) system_fault = 1;

    Display_Update();          /* 上电先画一帧 */
    OLED_RefreshAll();

#if USE_WATCHDOG
    Watchdog_Init();
#endif

    uint32_t last_ctrl = systick_cnt;
    uint32_t last_oled = systick_cnt - OLED_PERIOD_MS;  /* 立即刷新一次 */
    uint32_t last_volt = systick_cnt;
#if ENABLE_SONAR
    uint32_t sonic_time = 0;
    uint8_t  sonic_busy = 0;
#endif
    float    dt = 0.01f;

    while (1)
    {
        uint32_t now = systick_cnt;

#if USE_WATCHDOG
        Watchdog_Feed();
#endif

        /* ===== 控制拍：10ms ===== */
        if ((uint32_t)(now - last_ctrl) >= CTRL_PERIOD_MS)
        {
            uint32_t elapsed = now - last_ctrl;
            last_ctrl = now;

            dt = (float)elapsed * 0.001f;
            if (dt < 0.005f || dt > 0.050f) dt = 0.01f;

            Encoder_Update();
            MPU6050_GetData(dt);
            Balance_Calc(dt);

            /* 显示分页刷新：每拍最多搬一页，单次约 0.3ms */
            OLED_RefreshNext();
        }

        /* ===== 蓝牙：非阻塞轮询 + 失联保护 ===== */
        uint8_t cmd = Bluetooth_GetCmd();
        if (cmd != 0)
        {
            bt_last_cmd_ms = now;              /* 记录最后一次有效命令 */
            bt_link_active = 1;
            if (!obstacle_flag && !system_fault)
            {
                switch (cmd) {
                    case 'F': target_speed =  80; target_turn = 0;  break;
                    case 'B': target_speed = -80; target_turn = 0;  break;
                    case 'L': target_turn  = -30; break;
                    case 'R': target_turn  =  30; break;
                    case 'S': target_speed = 0;   target_turn = 0;  break;
                    default: break;
                }
            }
        }
        /* 断连保护：500ms 没有新命令就自动停车，避免断连后继续跑 */
        if (bt_link_active && (uint32_t)(now - bt_last_cmd_ms) > BT_CMD_TIMEOUT_MS)
        {
            bt_link_active = 0;
            target_speed = 0.0f;
            target_turn  = 0.0f;
        }

#if ENABLE_SONAR
        /* 60ms 一次：先发触发，35ms 后取结果（无回波则按无障碍处理） */
        if (!sonic_busy && (uint32_t)(now - sonic_time) >= SONAR_PERIOD_MS)
        {
            Hcsr04_StartMeasure();
            sonic_busy = 1;
            sonic_time = now;
        }
        if (sonic_busy && (uint32_t)(now - sonic_time) >= SONAR_WAIT_MS)
        {
            float d = Hcsr04_GetDistance_cm();

            if (d > 0.0f && d < OBSTACLE_STOP_CM)
            {
                obstacle_flag = 1;
                target_speed  = -60;
                target_turn   = 0;
            }
            else if (obstacle_flag && (d < 0.0f || d > OBSTACLE_CLEAR_CM))
            {
                obstacle_flag = 0;      /* 障碍物离开或超出量程 */
            }

            sonic_busy = 0;
        }
#endif

        /* ===== 显示：200ms 重建一帧显存 ===== */
        if ((uint32_t)(now - last_oled) >= OLED_PERIOD_MS)
        {
            last_oled = now;
            Display_Update();
        }

        /* ===== 低电压：只置标志，不阻塞 ===== */
        if ((uint32_t)(now - last_volt) >= VOLT_PERIOD_MS)
        {
            last_volt = now;

            float v = Get_Battery_Voltage();
            if (v < LOW_VOLTAGE_THRESHOLD) {
                if (low_volt_cnt < LOW_VOLTAGE_SAMPLES) low_volt_cnt++;
                if (low_volt_cnt >= LOW_VOLTAGE_SAMPLES) {
                    low_volt_latched = 1;
                    system_fault = 1;
                }
            } else {
                low_volt_cnt = 0;
            }
        }
    }
}

void SysTick_Handler(void)
{
    systick_cnt++;
}
