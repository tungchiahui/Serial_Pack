#ifndef WIRE_PROTOCOL_PROTOCOL_HPP_
#define WIRE_PROTOCOL_PROTOCOL_HPP_

#include <array>
#include <bit>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <span>
#include <tuple>
#include <type_traits>
#include <utility>

namespace wire_protocol
{

inline constexpr std::size_t kFrameOverhead = 7;
inline constexpr std::size_t kMaxPayloadSize = 100;

namespace detail
{

//======================================================
// 字段类型与线格式
//======================================================

template <typename T>
inline constexpr bool scalar = std::is_same_v<T, bool> ||
                               std::is_same_v<T, std::int8_t> ||
                               std::is_same_v<T, std::uint8_t> ||
                               std::is_same_v<T, std::int16_t> ||
                               std::is_same_v<T, std::uint16_t> ||
                               std::is_same_v<T, std::int32_t> ||
                               std::is_same_v<T, std::uint32_t> ||
                               std::is_same_v<T, float>;

template <typename T>
struct Field
{
    using Value = T;
    static constexpr bool supported = scalar<T>;
    static constexpr std::size_t count = 1;
};

template <typename T, std::size_t N>
struct Field<std::array<T, N>>
{
    using Value = T;
    static constexpr bool supported = scalar<T>;
    static constexpr std::size_t count = N;
};

template <typename T>
using Traits = Field<std::remove_cvref_t<T>>;

template <typename T>
constexpr int group() noexcept
{
    using Value = typename Traits<T>::Value;
    if constexpr (std::is_same_v<Value, bool>)
    {
        return 0;
    }
    else if constexpr (std::is_same_v<Value, float>)
    {
        return 4;
    }
    else if constexpr (sizeof(Value) == 1)
    {
        return 1;
    }
    else if constexpr (sizeof(Value) == 2)
    {
        return 2;
    }
    else
    {
        return 3;
    }
}

template <int Group, typename... T>
constexpr std::size_t count() noexcept
{
    return (std::size_t{0} + ... + (group<T>() == Group ? Traits<T>::count : 0));
}

template <typename... T>
inline constexpr std::size_t payload_size =
    (count<0, T...>() + 7) / 8 + count<1, T...>() +
    2 * count<2, T...>() + 4 * count<3, T...>() + 4 * count<4, T...>();

template <int Group, typename T, typename Function>
void visit(T& field, Function&& function) noexcept
{
    if constexpr (group<T>() == Group)
    {
        if constexpr (scalar<std::remove_cvref_t<T>>)
        {
            function(field);
        }
        else
        {
            for (auto& value : field)
            {
                function(value);
            }
        }
    }
}

inline std::uint16_t crc16(std::span<const std::uint8_t> bytes) noexcept
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

template <typename T>
void write(std::uint8_t*& cursor, T value) noexcept
{
    using Bits = std::conditional_t<sizeof(T) == 1, std::uint8_t,
                 std::conditional_t<sizeof(T) == 2, std::uint16_t, std::uint32_t>>;
    const auto bits = std::bit_cast<Bits>(value);
    for (std::size_t i = sizeof(T); i > 0; --i)
    {
        *cursor++ = static_cast<std::uint8_t>(bits >> ((i - 1) * 8));
    }
}

template <typename T>
void read(const std::uint8_t*& cursor, T& value) noexcept
{
    using Bits = std::conditional_t<sizeof(T) == 1, std::uint8_t,
                 std::conditional_t<sizeof(T) == 2, std::uint16_t, std::uint32_t>>;
    Bits bits = 0;
    for (std::size_t i = 0; i < sizeof(T); ++i)
    {
        bits = static_cast<Bits>((static_cast<std::uint32_t>(bits) << 8) | *cursor++);
    }
    value = std::bit_cast<T>(bits);
}

} // namespace detail

template <typename T>
concept WireField = detail::Traits<T>::supported;

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "wire_protocol requires IEEE 754 float32");

namespace detail
{

//======================================================
// 打包与解包：由 Protocol 调用
//======================================================

template <WireField... T>
[[nodiscard]] auto pack(std::uint8_t command, const T&... fields) noexcept
{
    constexpr auto size = payload_size<T...>;
    static_assert(size <= kMaxPayloadSize, "payload exceeds 100 bytes");

    std::array<std::uint8_t, size + kFrameOverhead> output{};
    output[0] = 0xA5;
    output[1] = 0x5A;
    output[2] = static_cast<std::uint8_t>(size);
    output[3] = command;

    std::size_t bit = 0;
    [[maybe_unused]] const auto write_bool = [&](bool value) noexcept
    {
        if (value)
        {
            output[4 + bit / 8] |= static_cast<std::uint8_t>(1U << (bit % 8));
        }
        ++bit;
    };
    (visit<0>(fields, write_bool), ...);

    auto* cursor = output.data() + 4 + (count<0, T...>() + 7) / 8;
    [[maybe_unused]] const auto write_value = [&](auto value) noexcept
    {
        write(cursor, value);
    };
    (visit<1>(fields, write_value), ...);
    (visit<2>(fields, write_value), ...);
    (visit<3>(fields, write_value), ...);
    (visit<4>(fields, write_value), ...);

    // CRC 保护长度、命令和数据；帧头、CRC 自身、帧尾不参与。
    const auto crc = crc16({output.data() + 2, size + 2});
    *cursor++ = static_cast<std::uint8_t>(crc >> 8);
    *cursor++ = static_cast<std::uint8_t>(crc);
    *cursor = 0xFF;
    return output;
}

// payload 仅在 Parser 回调期间有效。
struct FrameView
{
    std::uint8_t command{};
    std::span<const std::uint8_t> payload{};

    template <WireField... T>
        requires ((!std::is_const_v<T> && !std::is_volatile_v<T>) && ...)
    [[nodiscard]] bool unpack(T&... fields) const noexcept
    {
        constexpr auto size = payload_size<T...>;
        static_assert(size <= 255, "payload exceeds one-byte length");
        if (payload.size() != size)
        {
            return false;
        }
        if constexpr (size == 0)
        {
            return true;
        }

        std::size_t bit = 0;
        [[maybe_unused]] const auto read_bool = [&](bool& value) noexcept
        {
            value = (payload[bit / 8] & (1U << (bit % 8))) != 0;
            ++bit;
        };
        (visit<0>(fields, read_bool), ...);

        const auto* cursor = payload.data() + (count<0, T...>() + 7) / 8;
        [[maybe_unused]] const auto read_value = [&](auto& value) noexcept
        {
            read(cursor, value);
        };
        (visit<1>(fields, read_value), ...);
        (visit<2>(fields, read_value), ...);
        (visit<3>(fields, read_value), ...);
        (visit<4>(fields, read_value), ...);
        return true;
    }
};

//======================================================
// 串口字节流解析：固定容量、每路串口一个实例
//======================================================

template <std::size_t MaxPayload = kMaxPayloadSize>
class Parser
{
    static_assert(MaxPayload <= 255, "payload length is one byte");

  public:
    void reset() noexcept
    {
        size_ = 0;
    }

    template <typename OnFrame>
    void feed(std::span<const std::uint8_t> bytes, OnFrame&& on_frame)
    {
        for (const auto byte : bytes)
        {
            if (size_ == buffer_.size())
            {
                discard(1);
            }
            buffer_[size_++] = byte;

            while (size_ != 0)
            {
                if (buffer_[0] != 0xA5)
                {
                    discard(1);
                    continue;
                }
                if (size_ < 2)
                {
                    break;
                }
                if (buffer_[1] != 0x5A)
                {
                    discard(1);
                    continue;
                }
                if (size_ < 3)
                {
                    break;
                }

                const std::size_t length = buffer_[2];
                if (length > MaxPayload)
                {
                    discard(1);
                    continue;
                }
                const auto total = length + kFrameOverhead;
                if (size_ < total)
                {
                    break;
                }

                const auto payload =
                    std::span<const std::uint8_t>{buffer_.data() + 4, length};
                const auto received_crc = static_cast<std::uint16_t>(
                    (static_cast<std::uint16_t>(buffer_[length + 4]) << 8) |
                    buffer_[length + 5]);
                const auto expected_crc = crc16({buffer_.data() + 2, length + 2});
                if (buffer_[total - 1] != 0xFF || received_crc != expected_crc)
                {
                    discard(1);
                    continue;
                }

                on_frame(FrameView{buffer_[3], payload});
                discard(total);
            }
        }
    }

  private:
    void discard(std::size_t count) noexcept
    {
        size_ -= count;
        if (size_ != 0)
        {
            std::memmove(buffer_.data(), buffer_.data() + count, size_);
        }
    }

    std::array<std::uint8_t, MaxPayload + kFrameOverhead> buffer_{};
    std::size_t size_{};
};

//======================================================
// handler 签名提取：普通函数、成员函数、明确签名的 functor
//======================================================

template <typename>
inline constexpr bool unsupported_callable = false;

template <typename Callable, typename = void>
struct CallableTraits
{
    static_assert(unsupported_callable<Callable>,
                  "dispatch requires a non-generic callable with explicit parameter types");
};

template <typename Result, typename... Args>
struct CallableTraits<Result (*)(Args...), void>
{
    using Return = Result;
    using Arguments = std::tuple<Args...>;
};

template <typename Result, typename... Args>
struct CallableTraits<Result (*)(Args...) noexcept, void>
    : CallableTraits<Result (*)(Args...)>
{
};

#define WIRE_PROTOCOL_MEMBER_TRAITS(QUALIFIERS)                              \
    template <typename Result, typename Class, typename... Args>              \
    struct CallableTraits<Result (Class::*)(Args...) QUALIFIERS, void>         \
    {                                                                           \
        using Return = Result;                                                  \
        using Arguments = std::tuple<Args...>;                                 \
    }

WIRE_PROTOCOL_MEMBER_TRAITS();
WIRE_PROTOCOL_MEMBER_TRAITS(const);
WIRE_PROTOCOL_MEMBER_TRAITS(noexcept);
WIRE_PROTOCOL_MEMBER_TRAITS(const noexcept);

#undef WIRE_PROTOCOL_MEMBER_TRAITS

template <typename Callable>
struct CallableTraits<Callable,
                      std::void_t<decltype(&Callable::operator())>>
    : CallableTraits<decltype(&Callable::operator())>
{
};

template <typename Method, typename Object>
struct MemberHandler
{
    Method method;
    Object* object;

    template <typename... Args>
    void operator()(Args&&... args)
    {
        std::invoke(method, *object, std::forward<Args>(args)...);
    }
};

template <typename Method, typename Object>
struct CallableTraits<MemberHandler<Method, Object>, void> : CallableTraits<Method>
{
};

template <typename Tuple>
struct ValidArguments;

template <typename... Args>
struct ValidArguments<std::tuple<Args...>>
    : std::bool_constant<(... &&
        (std::is_same_v<Args, std::remove_cvref_t<Args>> && WireField<Args>))>
{
};

} // namespace detail

//======================================================
// 命令绑定：handler 参数即 payload schema
//======================================================

template <typename Callable>
class Dispatch
{
    using Signature = detail::CallableTraits<Callable>;
    using Arguments = typename Signature::Arguments;

    static_assert(std::is_same_v<typename Signature::Return, void>,
                  "dispatch handler must return void");
    static_assert(detail::ValidArguments<Arguments>::value,
                  "dispatch parameters must be supported wire fields passed by value");

  public:
    explicit Dispatch(std::uint8_t command, Callable handler)
        : command_(command), handler_(std::move(handler))
    {
    }

    [[nodiscard]] std::uint8_t command() const noexcept
    {
        return command_;
    }

    void handle(const detail::FrameView& frame)
    {
        handle_with_arguments(frame, std::type_identity<Arguments>{});
    }

  private:
    template <typename... Args>
    void handle_with_arguments(
        const detail::FrameView& frame,
        std::type_identity<std::tuple<Args...>>)
    {
        std::tuple<Args...> values{};
        const bool decoded = std::apply(
            [&](auto&... value) { return frame.unpack(value...); }, values);
        if (decoded)
        {
            std::apply(
                [&](auto&... value) { std::invoke(handler_, value...); }, values);
        }
    }

    std::uint8_t command_{};
    Callable handler_;
};

template <typename Callable>
[[nodiscard]] auto dispatch(std::uint8_t command, Callable&& handler)
{
    using Stored = std::decay_t<Callable>;
    return Dispatch<Stored>{command, std::forward<Callable>(handler)};
}

template <typename Method, typename Object>
    requires std::is_member_function_pointer_v<Method>
[[nodiscard]] auto dispatch(std::uint8_t command, Method method, Object* object)
{
    using Stored = detail::MemberHandler<Method, Object>;
    return Dispatch<Stored>{command, Stored{method, object}};
}

//======================================================
// 高层 API：发送、接收、重置
//======================================================

template <typename... Dispatches>
class Protocol
{
  public:
    explicit Protocol(Dispatches... dispatches)
        : dispatches_(std::move(dispatches)...)
    {
    }

    template <WireField... T>
    [[nodiscard]] auto pack(std::uint8_t command, const T&... fields) const noexcept
    {
        return detail::pack(command, fields...);
    }

    void feed(std::span<const std::uint8_t> bytes)
    {
        parser_.feed(bytes, [this](const detail::FrameView& frame)
        {
            bool handled = false;
            const auto visit = [&](auto& entry)
            {
                if (!handled && entry.command() == frame.command)
                {
                    handled = true;
                    entry.handle(frame);
                }
            };
            std::apply([&](auto&... entries) { (visit(entries), ...); }, dispatches_);
        });
    }

    void reset() noexcept
    {
        parser_.reset();
    }

  private:
    std::tuple<Dispatches...> dispatches_;
    detail::Parser<> parser_;
};

template <typename... Dispatches>
Protocol(Dispatches...) -> Protocol<Dispatches...>;

template <typename... Dispatches>
[[nodiscard]] auto make_protocol(Dispatches&&... dispatches)
{
    return Protocol<std::decay_t<Dispatches>...>{
        std::forward<Dispatches>(dispatches)...};
}

} // namespace wire_protocol

#endif // WIRE_PROTOCOL_PROTOCOL_HPP_
