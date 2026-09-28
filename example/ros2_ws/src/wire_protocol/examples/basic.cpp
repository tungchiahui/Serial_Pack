#include "wire_protocol/protocol.hpp"
#include <cstdio>

int main()
{
    const auto tx = wire_protocol::pack(0x01, std::uint32_t{42}, 1.0f, -2.0f, 0.5f);
    wire_protocol::Parser<> parser;
    const auto receive = [](const wire_protocol::Frame& frame) {
        if (frame.command == 0x01)
        {
            std::uint32_t seq{};
            float vx{}, vy{}, wz{};
            if (frame.unpack(seq, vx, vy, wz))
                std::printf("seq=%u vx=%.3f vy=%.3f wz=%.3f\n",
                            static_cast<unsigned>(seq), vx, vy, wz);
        }
    };
    // 模拟串口分两次收到一帧。
    parser.feed(std::span{tx}.first(5), receive);
    parser.feed(std::span{tx}.subspan(5), receive);
}
