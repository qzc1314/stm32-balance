#include "encoder.h"

static uint16_t last_cnt_a = 0;
static uint16_t last_cnt_b = 0;
static int16_t  delta_a    = 0;
static int16_t  delta_b    = 0;
static float    rpm_a      = 0.0f;
static float    rpm_b      = 0.0f;

/* 两通道都开输入滤波：f_DTS/32, N=8。
 * TIM_EncoderInterfaceConfig 不碰 CCMR 的 ICF 位，所以先调 TIM_ICInit 再调它。 */
static void Encoder_IC_Config(TIM_TypeDef *TIMx)
{
    TIM_ICInitTypeDef ic;

    TIM_ICStructInit(&ic);
    ic.TIM_Channel  = TIM_Channel_1;
    ic.TIM_ICFilter = 0x0F;
    TIM_ICInit(TIMx, &ic);

    ic.TIM_Channel  = TIM_Channel_2;
    ic.TIM_ICFilter = 0x0F;
    TIM_ICInit(TIMx, &ic);
}

void Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    /* 编码器引脚改用内部上拉：如果编码器是开漏输出（常见霍尔/光电编码器），
     * 板上又没有可靠的外部上拉，浮空输入只会读到噪声（表现为 ±1 乱跳）。
     * 内部上拉约 40k，对低速编码器足够；若板上确实有外部上拉，这样也安全。 */
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;   /* PA6/PA7 -> TIM3 */
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;   /* PB6/PB7 -> TIM4 */
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    TIM_TimeBaseStruct.TIM_Prescaler     = 0;
    TIM_TimeBaseStruct.TIM_Period        = 0xFFFF;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStruct);

    Encoder_IC_Config(TIM3);
    Encoder_IC_Config(TIM4);

    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_SetCounter(TIM3, 0);
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);

    last_cnt_a = 0;
    last_cnt_b = 0;
    delta_a = delta_b = 0;
}

/* 只允许在 10ms 节拍里调用一次。两个计数器在同一时刻取样，
 * 用无符号相减再转 int16，不受 0xFFFF 回绕影响。 */
void Encoder_Update(void)
{
    uint16_t now_a = TIM_GetCounter(TIM3);
    uint16_t now_b = TIM_GetCounter(TIM4);

    delta_a = (int16_t)(now_a - last_cnt_a);
    delta_b = (int16_t)(now_b - last_cnt_b);

    last_cnt_a = now_a;
    last_cnt_b = now_b;

    rpm_a = (float)delta_a * 60000.0f
          / ((float)SAMPLE_PERIOD_MS * (float)PULSE_PER_ROUND);
    rpm_b = (float)delta_b * 60000.0f
          / ((float)SAMPLE_PERIOD_MS * (float)PULSE_PER_ROUND);
}

int16_t EncoderA_GetDelta(void) { return delta_a; }
int16_t EncoderB_GetDelta(void) { return delta_b; }
float   EncoderA_GetSpeed_RPM(void) { return rpm_a; }
float   EncoderB_GetSpeed_RPM(void) { return rpm_b; }

