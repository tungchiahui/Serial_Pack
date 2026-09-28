#pragma once

#include <array>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <type_traits>

namespace wire_protocol
{
inline constexpr std::size_t kFrameOverhead = 7;
inline constexpr std::size_t kMaxPayloadSize = 100;

namespace detail
{
template <typename T>
inline constexpr bool scalar = std::is_same_v<T, bool> ||
    std::is_same_v<T, std::int8_t> || std::is_same_v<T, std::uint8_t> ||
    std::is_same_v<T, std::int16_t> || std::is_same_v<T, std::uint16_t> ||
    std::is_same_v<T, std::int32_t> || std::is_same_v<T, std::uint32_t> ||
    std::is_same_v<T, float>;

template <typename T> struct Field
{
    using Value = T;
    static constexpr bool supported = scalar<T>;
    static constexpr std::size_t count = 1;
};
template <typename T, std::size_t N> struct Field<std::array<T, N>>
{
    using Value = T;
    static constexpr bool supported = scalar<T>;
    static constexpr std::size_t count = N;
};
template <typename T> using Traits = Field<std::remove_cvref_t<T>>;

template <typename T> constexpr int group()
{
    using V = typename Traits<T>::Value;
    if constexpr (std::is_same_v<V, bool>) return 0;
    else if constexpr (std::is_same_v<V, float>) return 4;
    else if constexpr (sizeof(V) == 1) return 1;
    else if constexpr (sizeof(V) == 2) return 2;
    else return 3;
}

template <int Group, typename... T> constexpr std::size_t count()
{
    return (std::size_t{0} + ... + (group<T>() == Group ? Traits<T>::count : 0));
}
template <typename... T> inline constexpr std::size_t payload_size =
    (count<0, T...>() + 7) / 8 + count<1, T...>() +
    2 * count<2, T...>() + 4 * count<3, T...>() + 4 * count<4, T...>();

template <int Group, typename T, typename Function>
void visit(T& field, Function&& function) noexcept
{
    if constexpr (group<T>() == Group)
    {
        if constexpr (scalar<std::remove_cvref_t<T>>) function(field);
        else for (auto& value : field) function(value);
    }
}

inline std::uint16_t crc16(std::span<const std::uint8_t> bytes) noexcept
{
    std::uint16_t crc = 0xFFFF;
    for (const auto byte : bytes)
    {
        crc ^= byte;
        for (int bit = 0; bit < 8; ++bit)
            crc = static_cast<std::uint16_t>((crc >> 1) ^ ((crc & 1) ? 0xA001 : 0));
    }
    return crc;
}

template <typename T> void write(std::uint8_t*& cursor, T value) noexcept
{
    using U = std::conditional_t<sizeof(T) == 1, std::uint8_t,
        std::conditional_t<sizeof(T) == 2, std::uint16_t, std::uint32_t>>;
    const auto bits = std::bit_cast<U>(value);
    for (std::size_t i = sizeof(T); i > 0; --i)
        *cursor++ = static_cast<std::uint8_t>(bits >> ((i - 1) * 8));
}
template <typename T> void read(const std::uint8_t*& cursor, T& value) noexcept
{
    using U = std::conditional_t<sizeof(T) == 1, std::uint8_t,
        std::conditional_t<sizeof(T) == 2, std::uint16_t, std::uint32_t>>;
    U bits = 0;
    for (std::size_t i = 0; i < sizeof(T); ++i)
        bits = static_cast<U>((static_cast<std::uint32_t>(bits) << 8) | *cursor++);
    value = std::bit_cast<T>(bits);
}
} // namespace detail

template <typename T> concept WireField = detail::Traits<T>::supported;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "wire_protocol requires IEEE 754 float32");

// 返回拥有数据的定长数组；无堆内存，长度在编译期确定。
// 与旧版一致：bool(bit0 first), int8, int16, int32, float32；整数及浮点大端。
template <WireField... T>
[[nodiscard]] auto pack(std::uint8_t command, const T&... fields) noexcept
{
    constexpr auto size = detail::payload_size<T...>;
    static_assert(size <= kMaxPayloadSize, "payload exceeds 100 bytes");
    std::array<std::uint8_t, size + kFrameOverhead> output{};
    output[0] = 0xA5;
    output[1] = 0x5A;
    output[2] = static_cast<std::uint8_t>(size);
    output[3] = command;
    std::size_t bit = 0;
    [[maybe_unused]] const auto write_bool = [&](bool value) noexcept {
        if (value) output[4 + bit / 8] |= static_cast<std::uint8_t>(1U << (bit % 8));
        ++bit;
    };
    (detail::visit<0>(fields, write_bool), ...);
    auto* cursor = output.data() + 4 + (detail::count<0, T...>() + 7) / 8;
    [[maybe_unused]] const auto write_value = [&](auto value) noexcept { detail::write(cursor, value); };
    (detail::visit<1>(fields, write_value), ...);
    (detail::visit<2>(fields, write_value), ...);
    (detail::visit<3>(fields, write_value), ...);
    (detail::visit<4>(fields, write_value), ...);
    const auto crc = detail::crc16({output.data() + 4, size});
    *cursor++ = static_cast<std::uint8_t>(crc >> 8);
    *cursor++ = static_cast<std::uint8_t>(crc);
    *cursor = 0xFF;
    return output;
}

// 只读借用视图：仅在 Parser::feed 的回调期间有效。
struct Frame
{
    std::uint8_t command{};
    std::span<const std::uint8_t> payload{};

    // 按目标变量推导字段布局；长度不匹配返回 false，不修改任何变量。
    // 接收端必须与发送端约定相同的字段类型及各组内顺序。
    template <WireField... T>
        requires ((!std::is_const_v<T> && !std::is_volatile_v<T>) && ...)
    [[nodiscard]] bool unpack(T&... fields) const noexcept
    {
        constexpr auto size = detail::payload_size<T...>;
        static_assert(size <= 255, "payload exceeds one-byte length");
        if (payload.size() != size) return false;
        // 避免空 span 上的指针运算。
        if constexpr (size == 0) return true;
        std::size_t bit = 0;
        [[maybe_unused]] const auto read_bool = [&](bool& value) noexcept {
            value = (payload[bit / 8] & (1U << (bit % 8))) != 0;
            ++bit;
        };
        (detail::visit<0>(fields, read_bool), ...);
        const auto* cursor = payload.data() + (detail::count<0, T...>() + 7) / 8;
        [[maybe_unused]] const auto read_value = [&](auto& value) noexcept { detail::read(cursor, value); };
        (detail::visit<1>(fields, read_value), ...);
        (detail::visit<2>(fields, read_value), ...);
        (detail::visit<3>(fields, read_value), ...);
        (detail::visit<4>(fields, read_value), ...);
        return true;
    }
};

// 每路串口一个 Parser；固定容量，支持半包、粘包及坏帧后重新同步。
template <std::size_t MaxPayload = kMaxPayloadSize> class Parser
{
    static_assert(MaxPayload <= 255);
public:
    void reset() noexcept { size_ = 0; }

    // callback 无类型擦除/堆分配；不可在回调内再次 feed/reset 同一个解析器。
    template <typename OnFrame>
    void feed(std::span<const std::uint8_t> bytes, OnFrame&& on_frame)
    {
        for (auto byte : bytes)
        {
            if (size_ == buffer_.size()) discard(1);
            buffer_[size_++] = byte;
            while (size_ != 0)
            {
                if (buffer_[0] != 0xA5) { discard(1); continue; }
                if (size_ < 2) break;
                if (buffer_[1] != 0x5A) { discard(1); continue; }
                if (size_ < 3) break;
                const std::size_t length = buffer_[2];
                if (length > MaxPayload) { discard(1); continue; }
                const auto total = length + kFrameOverhead;
                if (size_ < total) break;
                const auto payload = std::span<const std::uint8_t>{buffer_.data() + 4, length};
                const auto crc = static_cast<std::uint16_t>(
                    (static_cast<std::uint16_t>(buffer_[length + 4]) << 8) | buffer_[length + 5]);
                if (buffer_[total - 1] != 0xFF || crc != detail::crc16(payload))
                { discard(1); continue; }
                on_frame(Frame{buffer_[3], payload});
                discard(total);
            }
        }
    }
private:
    void discard(std::size_t count) noexcept
    {
        size_ -= count;
        if (size_ != 0) std::memmove(buffer_.data(), buffer_.data() + count, size_);
    }
    std::array<std::uint8_t, MaxPayload + kFrameOverhead> buffer_{};
    std::size_t size_{};
};
} // namespace wire_protocol
