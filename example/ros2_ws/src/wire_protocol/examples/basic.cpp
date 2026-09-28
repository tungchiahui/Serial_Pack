#include "wire_protocol/protocol.hpp"

#include <cstdio>

int main()
{
    auto protocol = wire_protocol::make_protocol(
        wire_protocol::dispatch(0x01,
            [](std::uint32_t seq, float vx, float vy, float wz)
            {
                std::printf("seq=%u vx=%.3f vy=%.3f wz=%.3f\n",
                            static_cast<unsigned>(seq), vx, vy, wz);
            }));

    const auto tx = protocol.pack(0x01, std::uint32_t{42}, 1.0f, -2.0f, 0.5f);
    // 模拟串口分两次收到一帧。
    protocol.feed(std::span{tx}.first(5));
    protocol.feed(std::span{tx}.subspan(5));
}
