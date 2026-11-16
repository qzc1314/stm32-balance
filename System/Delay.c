#include "stm32f10x.h"
#include "delay.h"

/* SysTick 已由 SysTick_Config(SystemCoreClock/1000) 配好：
 *   LOAD = 71999，1ms 一次中断，CTRL.ENABLE=1、TICKINT=1。
 * 这里只读 VAL 做计数，绝不写 LOAD / CTRL / VAL，
 * 所以不会影响 1ms 节拍，也不会把中断关掉。
 *
 * 计数值从 LOAD 递减到 0 再重装，用无符号差值累加，
 * 单次延时可大于 1ms（多次回绕也能正确累计）。 */

void Delay_us(uint32_t xus)
{
    uint32_t ticks;
    uint32_t last, now, elapsed = 0;

    if (xus == 0) return;

    ticks = xus * (SystemCoreClock / 1000000U);   /* 72MHz -> 72 tick/us */
    last  = SysTick->VAL;

    while (elapsed < ticks)
    {
        now = SysTick->VAL;

        if (now <= last)
            elapsed += last - now;                     /* 未回绕 */
        else
            elapsed += last + (SysTick->LOAD + 1U) - now;   /* 回绕 */

        last = now;
    }
}

void Delay_ms(uint32_t xms)
{
    while (xms--) Delay_us(1000);
}

void Delay_s(uint32_t xs)
{
    while (xs--) Delay_ms(1000);
}


