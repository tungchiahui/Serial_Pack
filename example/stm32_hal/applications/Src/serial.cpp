#include "serial.hpp"
#include "protocol.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include "cmsis_os2.h"
#include "main.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_uart.h"
#include "struct_typedef.h"
#include "usart.h"

std::array<uint8_t, 1> rx_buffer;

wire_protocol::CallbackProtocol protocol_;

//模拟数据
uint64_t cmd_count_{0};

struct
{
    uint32_t seq;
    std::array<fp32, 3> v;
}cmd_vel;

struct
{
    uint32_t seq;
    int32_t mode;
}set_mode;


void cmd_vel_callback(uint32_t seq, fp32 vx, fp32 vy, fp32 wz)
{
    cmd_vel.seq = seq;
    cmd_vel.v[0] = vx;
    cmd_vel.v[1] = vy;
    cmd_vel.v[2] = wz;
}


void set_mode_callback(uint32_t seq, int32_t mode)
{
    set_mode.seq = seq;
    set_mode.mode = mode;
}

extern "C"
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        protocol_.feed(rx_buffer);
        
        //开启发送中断
        HAL_UART_Receive_IT(&huart1, rx_buffer.data(), rx_buffer.size());

    }
}


extern "C"
void StartSerialTask(void *argument)
{
    //开启解包
    if(!protocol_.set_unpack_callback(0x01,cmd_vel_callback) || 
        !protocol_.set_unpack_callback(0x02,set_mode_callback)
    )
    {
        Error_Handler();
    }

    //开启发送中断
    HAL_UART_Receive_IT(&huart1, rx_buffer.data(), rx_buffer.size());

    for(;;)
    {
        // 模拟不断变化的速度命令
        ++cmd_count_;

        fp64 t = cmd_count_ * 0.01;

        uint32_t seq = static_cast<uint32_t>(cmd_count_);

        fp32 vx = static_cast<fp32>(std::sin(t));
        fp32 vy = static_cast<fp32>(std::cos(t));
        fp32 wz = static_cast<fp32>(0.5 * std::sin(t));

        auto frame = protocol_.pack(0x01,seq,vx,vy,wz);

        HAL_UART_Transmit(&huart1, frame.data(), frame.size(), osWaitForever);

        osDelay(1);
    }
}

extern "C"
void StartLedTask(void *argument)
{
    for (;;) 
    {
        HAL_GPIO_TogglePin(BSP_LED_GPIO_Port, BSP_LED_Pin);
        osDelay(500);
    }
}