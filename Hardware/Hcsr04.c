#include "stm32f10x.h"                  // Device header
#include "Hcsr04.h"
#include "Delay.h"

#define TRIG_PORT   GPIOA
#define TRIG_PIN    GPIO_Pin_0
#define ECHO_PORT   GPIOA
#define ECHO_PIN    GPIO_Pin_1   // 复用为 TIM2_CH2

// 定时器计数频率 1MHz → 1us 分辨率
#define TIM2_PSC    71           // 72MHz / (71+1) = 1MHz

/* 量程判定：58us/cm。4m 回波约 23.2ms，旧的 4000 上限实际只有 69cm，
 * 这里放宽到 25ms（约 4.3m），低于 100us 视为近距盲区。 */
#define ECHO_MIN_US   100UL
#define ECHO_MAX_US   25000UL

// 测量状态
static volatile uint32_t capture_rising  = 0;
static volatile uint32_t capture_falling = 0;
static volatile uint8_t  capture_flag    = 0;  // 0:空闲 1:已捕获上升沿
static volatile uint32_t overflow_count  = 0;
static float last_distance = -1.0f;

/* 复位捕获状态机：上一次测量没有回波时，capture_flag 会停在 1，
 * 若不清理，下一次的上升沿会被当成下降沿，算出错误的距离。 */
static void Capture_Reset(void)
{
    TIM_ITConfig(TIM2, TIM_IT_CC2, DISABLE);
    capture_flag    = 0;
    overflow_count  = 0;
    capture_rising  = 0;
    capture_falling = 0;
    TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Rising);
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2 | TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
}

/**
 * @brief  HC-SR04 初始化
 *         TRIG=PA0 推挽输出
 *         ECHO=PA1 浮空输入，复用 TIM2_CH2 输入捕获
 */
void Hcsr04_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_ICInitTypeDef TIM_ICInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 1. 时钟使能
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   // TIM2 在 APB1

    // 2. TRIG 引脚：普通推挽输出
    GPIO_InitStruct.GPIO_Pin   = TRIG_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TRIG_PORT, &GPIO_InitStruct);
    GPIO_ResetBits(TRIG_PORT, TRIG_PIN);

    // 3. ECHO 引脚：浮空输入（输入捕获模式下仍配置为浮空输入）
    GPIO_InitStruct.GPIO_Pin   = ECHO_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ECHO_PORT, &GPIO_InitStruct);

    // 4. 定时器 TIM2 时基：计数频率 1MHz，自动重载 65535
    TIM_TimeBaseStruct.TIM_Prescaler         = TIM2_PSC;
    TIM_TimeBaseStruct.TIM_Period            = 65535;
    TIM_TimeBaseStruct.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);

    // 5. 输入捕获通道 2（对应 PA1）
    TIM_ICInitStruct.TIM_Channel     = TIM_Channel_2;
    TIM_ICInitStruct.TIM_ICPolarity  = TIM_ICPolarity_Rising;   // 先捕获上升沿
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStruct.TIM_ICFilter    = 0x0;                     // 无滤波
    TIM_ICInit(TIM2, &TIM_ICInitStruct);

    // 6. 中断使能：捕获中断 + 更新中断（处理定时器溢出）
    TIM_ITConfig(TIM2, TIM_IT_CC2 | TIM_IT_Update, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel    = TIM2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    // 7. 启动定时器
    TIM_Cmd(TIM2, ENABLE);
    Capture_Reset();
}

/**
 * @brief  触发一次测量（发送 >=10us 高脉冲）
 * @note   原实现用 13 个 NOP 当 1us，实际只有约 0.2us，模块不会响应；
 *         这里改用 SysTick 的 Delay_us，并保证先清掉上一次的残留状态。
 */
void Hcsr04_StartMeasure(void)
{
    Capture_Reset();
    GPIO_SetBits(TRIG_PORT, TRIG_PIN);
    Delay_us(12);
    GPIO_ResetBits(TRIG_PORT, TRIG_PIN);
}

/**
 * @brief  放弃当前测量（超时无回波时调用），重置状态机
 */
void Hcsr04_Reset(void)
{
    Capture_Reset();
}

/**
 * @brief  返回最近一次测量的距离（cm），无有效数据时返回 -1
 */
float Hcsr04_GetDistance_cm(void)
{
    return last_distance;
}

/* TIM2 中断服务函数 */
void TIM2_IRQHandler(void)
{
    // 溢出中断
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        overflow_count++;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }

    // 通道2捕获事件
    if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET) {
        uint32_t val = TIM_GetCapture2(TIM2);

        if (capture_flag == 0) {               // 上升沿
            capture_rising = val;
            overflow_count = 0;
            // 切换为下降沿捕获
            TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Falling);
            capture_flag = 1;
        }
        else {                                 // 下降沿
            capture_falling = val;
            // 总计数 = 溢出次数 * 65536 + (下降沿 - 上升沿)，无符号运算自动处理回绕
            uint32_t total_ticks = (overflow_count * 65536UL)
                                 + capture_falling - capture_rising;

            if (total_ticks >= ECHO_MIN_US && total_ticks <= ECHO_MAX_US) {
                last_distance = (float)total_ticks / 58.0f;
            } else {
                last_distance = -1.0f;         // 超程/异常，按无回波处理
            }

            // 恢复上升沿捕获，为下一次测量准备
            TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Rising);
            capture_flag = 0;  // 回到空闲状态
        }

        TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
    }
}
