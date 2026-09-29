#include "bsp_delay.hpp"
#include "main.h"

namespace
{
uint32_t cycles_per_us = 0;

void init_cycle_counter(uint16_t sysclk_mhz)
{
    cycles_per_us = sysclk_mhz;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void busy_wait_us(uint32_t duration_us)
{
    if (cycles_per_us == 0U)
    {
        init_cycle_counter(static_cast<uint16_t>(SystemCoreClock / 1000000U));
    }

    // 分段等待，避免每段所需的周期数超过 32 位范围。
    while (duration_us != 0U)
    {
        const uint32_t step_us = duration_us > 1000U ? 1000U : duration_us;
        const uint32_t start = DWT->CYCCNT;
        const uint32_t cycles = step_us * cycles_per_us;
        while (static_cast<uint32_t>(DWT->CYCCNT - start) < cycles)
        {
        }
        duration_us -= step_us;
    }
}

void busy_wait_ms(uint16_t duration_ms)
{
    while (duration_ms-- != 0U)
    {
        busy_wait_us(1000U);
    }
}
}

BSP_Delay bsp_delay;

void BSP_Delay::F1::Init(uint16_t sysclk)
{
    init_cycle_counter(sysclk);
}

void BSP_Delay::F1::us(uint32_t nus)
{
    busy_wait_us(nus);
}

void BSP_Delay::F1::ms(uint16_t nms)
{
    busy_wait_ms(nms);
}

void BSP_Delay::F4::Init(uint16_t sysclk)
{
    init_cycle_counter(sysclk);
}

void BSP_Delay::F4::us(uint32_t nus)
{
    busy_wait_us(nus);
}

void BSP_Delay::F4::ms(uint16_t nms)
{
    busy_wait_ms(nms);
}

void BSP_Delay::FreeRTOS::Init(void)
{
    // 保留此接口以兼容原有代码；SysTick 由调度器管理。
}
