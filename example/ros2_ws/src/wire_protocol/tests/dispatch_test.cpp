#include "wire_protocol/protocol.hpp"

#include <cstdio>
#include <cstdlib>
#include <new>

void* operator new(std::size_t) { std::abort(); }
void* operator new[](std::size_t) { std::abort(); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

#define CHECK(...) do { if (!(__VA_ARGS__)) { \
    std::fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #__VA_ARGS__); \
    std::abort(); } } while (false)

namespace
{
int function_calls = 0;
void normal_function(std::uint32_t seq, float x)
{
    CHECK(seq == 42 && x == 1.25f);
    ++function_calls;
}
void no_throw_function(std::uint8_t value) noexcept
{
    CHECK(value == 7);
    ++function_calls;
}

struct Receiver
{
    int mode_calls = 0;
    mutable int const_calls = 0;
    int no_throw_calls = 0;
    mutable int const_no_throw_calls = 0;

    void mode(std::uint32_t seq, std::int32_t mode)
    {
        CHECK(seq == 9 && mode == -3);
        ++mode_calls;
    }
    void observe(std::uint16_t value) const
    {
        CHECK(value == 0xA123);
        ++const_calls;
    }
    void safe(std::int8_t value) noexcept
    {
        CHECK(value == -8);
        ++no_throw_calls;
    }
    void safe_observe(bool value) const noexcept
    {
        CHECK(value);
        ++const_no_throw_calls;
    }
};

struct Mixed
{
    int calls = 0;
    void handle(bool enabled, std::uint8_t u8, std::int8_t i8,
                std::uint16_t u16, std::int16_t i16,
                std::uint32_t u32, std::int32_t i32, float value,
                std::array<std::int16_t, 2> samples)
    {
        CHECK(enabled && u8 == 255 && i8 == -128);
        CHECK(u16 == 65000 && i16 == -1234);
        CHECK(u32 == 0xFEDCBA98 && i32 == -87654321);
        CHECK(std::bit_cast<std::uint32_t>(value) == 0x80000000);
        CHECK(samples[0] == -32768 && samples[1] == 32767);
        ++calls;
    }
};
}

int main()
{
    Receiver receiver;
    Mixed mixed;
    int lambda_calls = 0;
    int empty_calls = 0;
    int max_calls = 0;
    const Receiver* const_receiver = &receiver;

    auto protocol = wire_protocol::make_protocol(
        wire_protocol::dispatch(0x01, normal_function),
        wire_protocol::dispatch(0x02, &Receiver::mode, &receiver),
        wire_protocol::dispatch(0x03,
            [&lambda_calls](std::uint32_t seq, float x)
            {
                CHECK(seq == 10 && x == -2.5f);
                ++lambda_calls;
            }),
        wire_protocol::dispatch(0x04, &Receiver::observe, const_receiver),
        wire_protocol::dispatch(0x05, &Receiver::safe, &receiver),
        wire_protocol::dispatch(0x06, &Receiver::safe_observe, const_receiver),
        wire_protocol::dispatch(0x07, no_throw_function),
        wire_protocol::dispatch(0x08, &Mixed::handle, &mixed),
        wire_protocol::dispatch(0x09, [&empty_calls]() { ++empty_calls; }),
        wire_protocol::dispatch(0x0A, [&max_calls](std::array<std::uint8_t, 100> bytes)
        {
            for (auto byte : bytes) CHECK(byte == 0x5A);
            ++max_calls;
        }));

    const auto a = protocol.pack(0x01, std::uint32_t{42}, 1.25f);
    const auto b = protocol.pack(0x02, std::uint32_t{9}, std::int32_t{-3});
    const auto c = protocol.pack(0x03, std::uint32_t{10}, -2.5f);
    static_assert(a.size() == 15);
    CHECK(a[2] == 8 && a[3] == 0x01);
    CHECK(a[a.size() - 3] ==
          static_cast<std::uint8_t>(wire_protocol::detail::crc16(
              std::span{a}.subspan(2, a[2] + 2)) >> 8));

    // 分包和粘包中的三种 callable 按顺序收到。
    protocol.feed(std::span{a}.first(3));
    CHECK(function_calls == 0);
    std::array<std::uint8_t, a.size() - 3 + b.size() + c.size()> combined{};
    std::size_t position = 0;
    for (auto byte : std::span{a}.subspan(3)) combined[position++] = byte;
    for (auto byte : b) combined[position++] = byte;
    for (auto byte : c) combined[position++] = byte;
    protocol.feed(combined);
    CHECK(function_calls == 1 && receiver.mode_calls == 1 && lambda_calls == 1);

    // 合法但未注册的命令、错误 schema、坏 CRC/命令/帧尾都不会调用 handler。
    const auto unknown = protocol.pack(0x99, std::uint8_t{3});
    protocol.feed(unknown);
    const auto short_payload = protocol.pack(0x01, std::uint32_t{42});
    protocol.feed(short_payload);
    auto bad_crc = a;
    bad_crc[4] ^= 0x01;
    protocol.feed(bad_crc);
    auto bad_command = a;
    bad_command[3] = 0x02;
    protocol.feed(bad_command);
    auto bad_length = a;
    --bad_length[2];
    protocol.feed(bad_length);
    auto bad_tail = a;
    bad_tail.back() = 0x00;
    protocol.feed(bad_tail);
    CHECK(function_calls == 1 && receiver.mode_calls == 1);
    protocol.feed(a);
    CHECK(function_calls == 2);

    const auto no_throw = protocol.pack(0x07, std::uint8_t{7});
    protocol.feed(no_throw);
    CHECK(function_calls == 3);
    protocol.feed(protocol.pack(0x04, std::uint16_t{0xA123}));
    protocol.feed(protocol.pack(0x05, std::int8_t{-8}));
    protocol.feed(protocol.pack(0x06, true));
    CHECK(receiver.const_calls == 1 && receiver.no_throw_calls == 1 &&
          receiver.const_no_throw_calls == 1);

    const std::array<std::int16_t, 2> samples{-32768, 32767};
    const auto mixed_frame = protocol.pack(
        0x08, std::bit_cast<float>(std::uint32_t{0x80000000}),
        std::uint32_t{0xFEDCBA98}, true, std::uint8_t{255},
        std::int8_t{-128}, std::uint16_t{65000}, std::int16_t{-1234},
        samples, std::int32_t{-87654321});
    protocol.feed(mixed_frame);
    CHECK(mixed.calls == 1);

    protocol.feed(protocol.pack(0x09));
    CHECK(empty_calls == 1);
    std::array<std::uint8_t, 100> maximum{};
    maximum.fill(0x5A);
    protocol.feed(protocol.pack(0x0A, maximum));
    CHECK(max_calls == 1);

    protocol.feed(std::span{a}.first(5));
    protocol.reset();
    protocol.feed(a);
    CHECK(function_calls == 4);

    // 无显式类型擦除；类成员可用静态 helper 推导声明类型。
    struct Owner
    {
        int calls = 0;
        void receive(std::uint8_t) { ++calls; }
        using ProtocolType = decltype(wire_protocol::make_protocol(
            wire_protocol::dispatch(0x30, &Owner::receive,
                                    static_cast<Owner*>(nullptr))));
        ProtocolType instance = wire_protocol::make_protocol(
            wire_protocol::dispatch(0x30, &Owner::receive, this));
    };
    Owner owner;
    owner.instance.feed(owner.instance.pack(0x30, std::uint8_t{1}));
    CHECK(owner.calls == 1);
    std::puts("dispatch tests passed (heap allocation forbidden)");
}
