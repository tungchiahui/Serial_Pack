#include "wire_protocol/protocol.hpp"
#include <cstdio>
#include <cstdlib>
#include <new>

// 任意普通 C++ 堆分配都会使测试失败，覆盖打包/解析/解包路径。
void* operator new(std::size_t) { std::abort(); }
void* operator new[](std::size_t) { std::abort(); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

#define CHECK(...) do { if (!(__VA_ARGS__)) { \
    std::fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #__VA_ARGS__); \
    std::abort(); } } while (false)

int main()
{
    using namespace wire_protocol;
    using detail::Parser;
    using Frame = detail::FrameView;
    Protocol<> protocol;
    const auto pack = [&](std::uint8_t command, const auto&... fields)
    {
        return protocol.pack(command, fields...);
    };
    constexpr std::array<std::uint8_t, 9> crc_input{'1','2','3','4','5','6','7','8','9'};
    CHECK(detail::crc16(crc_input) == 0x4B37);
    const std::uint32_t seq = 0xFEDCBA98;
    const auto tx = pack(0x01, seq, 1.0f, -2.0f, 0.5f);
    static_assert(tx.size() == 23);
    CHECK(tx[4] == 0xFE && tx[5] == 0xDC && tx[6] == 0xBA && tx[7] == 0x98);
    CHECK(tx[8] == 0x3F && tx[9] == 0x80 && tx[12] == 0xC0);
    const auto crc = detail::crc16(std::span{tx}.subspan(2, tx[2] + 2));
    CHECK(tx[tx.size() - 3] == static_cast<std::uint8_t>(crc >> 8));
    CHECK(tx[tx.size() - 2] == static_cast<std::uint8_t>(crc));
    CHECK(crc != detail::crc16(std::span{tx}.subspan(4, tx[2])));
    // 旧版 README 的实际发送帧：字段及帧结构保持，CRC 改为覆盖 LEN+CMD+DATA。
    const auto upgraded = pack(0x01, std::uint32_t{1}, 1.0f, 0.5f, -0.5f);
    constexpr std::array<std::uint8_t, 23> expected{
        0xA5, 0x5A, 0x10, 0x01, 0x00, 0x00, 0x00, 0x01,
        0x3F, 0x80, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00,
        0xBF, 0x00, 0x00, 0x00, 0x26, 0xBD, 0xFF};
    CHECK(upgraded == expected);
    auto old_frame = expected;
    old_frame[20] = 0x67;
    old_frame[21] = 0x27;
    int old_frame_received = 0;
    Parser<> compatibility_parser;
    compatibility_parser.feed(old_frame, [&](const Frame&) { ++old_frame_received; });
    CHECK(old_frame_received == 0);
    Parser<> parser;
    int received = 0;
    const auto receive = [&](const Frame& frame) {
        std::uint32_t decoded{};
        float vx{}, vy{}, wz{};
        CHECK(frame.command == 0x01);
        CHECK(frame.unpack(decoded, vx, vy, wz));
        CHECK(decoded == seq && vx == 1.0f && vy == -2.0f && wz == 0.5f);
        std::int32_t unchanged = 123;
        CHECK(!frame.unpack(unchanged));
        CHECK(unchanged == 123);
        ++received;
    };
    // 所有两段切分位置，包括帧头、payload、CRC 中间。
    for (std::size_t split = 0; split <= tx.size(); ++split)
    {
        parser.reset();
        const auto before = received;
        parser.feed(std::span{tx}.first(split), receive);
        parser.feed(std::span{tx}.subspan(split), receive);
        CHECK(received == before + 1);
    }
    std::array<std::uint8_t, tx.size() * 2> joined{};
    for (std::size_t i = 0; i < tx.size(); ++i) joined[i] = joined[i + tx.size()] = tx[i];
    const auto before = received;
    parser.feed(joined, receive);
    CHECK(received == before + 2);
    constexpr std::array<std::uint8_t, 7> noise{0, 0xFF, 0xA5, 0xA5, 0x5A, 255, 0};
    parser.feed(noise, receive);
    auto damaged = tx;
    damaged[8] ^= 1;
    parser.feed(damaged, receive);
    damaged = tx;
    damaged.back() = 0;
    parser.feed(damaged, receive);
    parser.feed(tx, receive);
    CHECK(received == before + 3);
    parser.feed(std::span{tx}.first(5), receive);
    parser.reset();
    parser.feed(tx, receive);
    CHECK(received == before + 4);
    auto wrong_command = tx;
    wrong_command[3] = 0x02;
    parser.feed(wrong_command, receive);
    CHECK(received == before + 4);

    const std::array<bool, 9> flags{true, false, true, true, false, false, false, true, true};
    const std::array<std::int16_t, 2> shorts{-32768, 32767};
    const auto mixed = pack(0x03, -0.0f, seq, flags, std::int8_t{-128}, shorts,
                            std::uint8_t{255}, std::uint16_t{65535}, std::int32_t{-2147483647});
    CHECK(mixed[4] == 0x8D && mixed[5] == 1);
    int mixed_received = 0;
    parser.feed(mixed, [&](const Frame& frame) {
        std::array<bool, 9> b{};
        std::array<std::int16_t, 2> s{};
        std::int8_t i8{};
        std::uint8_t u8{};
        std::uint16_t u16{};
        std::int32_t i32{};
        std::uint32_t u32{};
        float f{};
        CHECK(frame.unpack(f, u32, b, i8, s, u8, u16, i32));
        CHECK(b == flags && s == shorts && i8 == -128 && u8 == 255 && u16 == 65535);
        CHECK(u32 == seq && i32 == -2147483647 && std::bit_cast<std::uint32_t>(f) == 0x80000000);
        ++mixed_received;
    });
    CHECK(mixed_received == 1);
    const auto empty = pack(0x04);
    Parser<0> empty_parser;
    int empty_received = 0;
    empty_parser.feed(empty, [&](const Frame& frame) {
        CHECK(frame.command == 0x04 && frame.unpack());
        ++empty_received;
    });
    CHECK(empty_received == 1);
    const std::array<std::uint8_t, 100> maximum{};
    const auto full = pack(0x05, maximum);
    int full_received = 0;
    parser.feed(full, [&](const Frame& frame) {
        std::array<std::uint8_t, 100> decoded{};
        CHECK(frame.unpack(decoded) && decoded == maximum);
        ++full_received;
    });
    CHECK(full_received == 1);
    // 特殊 float 位模式必须原样保留，包括 NaN/无穷。
    const std::array<float, 2> specials{
        std::bit_cast<float>(std::uint32_t{0x7FC12345}),
        std::bit_cast<float>(std::uint32_t{0x7F800000})};
    const auto special_tx = pack(0x06, specials);
    parser.feed(special_tx, [&](const Frame& frame) {
        std::array<float, 2> decoded{};
        CHECK(frame.unpack(decoded));
        CHECK(std::bit_cast<std::uint32_t>(decoded[0]) == 0x7FC12345);
        CHECK(std::bit_cast<std::uint32_t>(decoded[1]) == 0x7F800000);
    });
    std::puts("wire_protocol tests passed (heap allocation forbidden)");
}
