#include "wire_protocol/protocol.hpp"

namespace wire_protocol::detail
{

//======================================================
// CRC16：初值 0xFFFF，反射多项式 0xA001
//======================================================

std::uint16_t crc16(std::span<const std::uint8_t> bytes) noexcept
{
    std::uint16_t crc = 0xFFFF;
    for (const auto byte : bytes)
    {
        crc ^= byte;
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = static_cast<std::uint16_t>(
                (crc >> 1) ^ ((crc & 1) ? 0xA001 : 0));
        }
    }
    return crc;
}

} // namespace wire_protocol::detail
