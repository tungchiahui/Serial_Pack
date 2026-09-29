#ifndef BSP_DELAY_H
#define BSP_DELAY_H

#include <stdint.h>

// 仅供 C++ 使用；sysclk 是以 MHz 为单位的 CPU 时钟频率。
class BSP_Delay
{
public:
    class F1
    {
    public:
        void Init(uint16_t sysclk);
        void us(uint32_t nus);
        void ms(uint16_t nms);
    } f1;

    class F4
    {
    public:
        void Init(uint16_t sysclk);
        void us(uint32_t nus);
        void ms(uint16_t nms);
    } f4;

    class FreeRTOS
    {
    public:
        void Init(void);
    } freertos;
};

extern BSP_Delay bsp_delay;

#endif
