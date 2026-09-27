#include "wire_protocol/protocol.hpp"

#include <bit>
#include <cstring>
#include <limits>

namespace wire_protocol
{
namespace
{

static_assert(sizeof(float) == sizeof(std::uint32_t));
static_assert(std::numeric_limits<float>::is_iec559);


/**
 * @brief       检查各类型字段数量是否在接收端允许的范围内。
 * @param       counts: 各类型字段的数量。
 * @retval      true: 数量均有效；false: 至少一种字段数量超出上限。
 */
bool counts_valid(const FieldCounts &counts) noexcept
{
    return counts.bools <= kMaxBoolCount && counts.int8s <= kMaxInt8Count &&
           counts.int16s <= kMaxInt16Count && counts.int32s <= kMaxInt32Count &&
           counts.float32s <= kMaxFloat32Count;
}

/**
 * @brief       计算各字段序列化后的数据区长度。
 * @param       counts: 各类型字段的数量。
 * @note        布尔值每 8 个打包为 1 字节，其余字段按类型字节数计算。
 * @retval      数据区长度，单位为字节。
 */
std::size_t payload_size(const FieldCounts &counts) noexcept
{
    return (counts.bools + 7) / 8 + counts.int8s + 2 * counts.int16s + 4 * counts.int32s +
           4 * counts.float32s;
}

/**
 * @brief       从字段视图中取得各类型字段的数量。
 * @param       fields: 待统计的字段视图。
 * @retval      各类型字段的数量。
 */
FieldCounts counts_of(const FieldSpans &fields) noexcept
{
    return {fields.bools.size(), fields.int8s.size(), fields.int16s.size(), fields.int32s.size(),
            fields.float32s.size()};
}

/**
 * @brief       检查发送数据区长度并计算其大小。
 * @param       counts: 各类型字段的数量。
 * @note        先检查各项大小，避免计算总长度时发生整数溢出。
 * @retval      有效时返回数据区长度；超过发送上限时返回 std::nullopt。
 */
std::optional<std::size_t> tx_payload_size(const FieldCounts &counts) noexcept
{
    // 原始代码的发送限制取决于数据区总字节数，而不是接收数组各自的容量。
    // 在进行乘法计算前先检查各项，避免整数溢出。

    //只要一个溢出，就都溢出。
    if (counts.bools > kMaxTxPayloadSize * 8 || counts.int8s > kMaxTxPayloadSize ||
        counts.int16s > kMaxTxPayloadSize / 2 || counts.int32s > kMaxTxPayloadSize / 4 ||
        counts.float32s > kMaxTxPayloadSize / 4)
    {
        return std::nullopt;
    }

    //获取序列化后的长度（打包）
    const std::size_t length = payload_size(counts);

    //总长度溢出
    if (length > kMaxTxPayloadSize)
    {
        return std::nullopt;
    }

    return length;
}

/**
 * @brief       按大端字节序写入 16 位无符号整数。
 * @param       value: 待写入的数值。
 * @param       cursor: 写入位置；写入后向前移动 2 字节。
 * @retval      无。
 */
void write_u16_be(std::uint16_t value, std::uint8_t *&cursor) noexcept
{
    *cursor++ = static_cast<std::uint8_t>(value >> 8);
    *cursor++ = static_cast<std::uint8_t>(value);
}

/**
 * @brief       按大端字节序写入 32 位无符号整数。
 * @param       value: 待写入的数值。
 * @param       cursor: 写入位置；写入后向前移动 4 字节。
 * @retval      无。
 */
void write_u32_be(std::uint32_t value, std::uint8_t *&cursor) noexcept
{
    *cursor++ = static_cast<std::uint8_t>(value >> 24);
    *cursor++ = static_cast<std::uint8_t>(value >> 16);
    *cursor++ = static_cast<std::uint8_t>(value >> 8);
    *cursor++ = static_cast<std::uint8_t>(value);
}

/**
 * @brief       按大端字节序读取 16 位无符号整数。
 * @param       cursor: 读取位置；读取后向前移动 2 字节。
 * @retval      读取到的 16 位无符号整数。
 */
std::uint16_t read_u16_be(const std::uint8_t *&cursor) noexcept
{
    const auto result =
        static_cast<std::uint16_t>((static_cast<std::uint16_t>(cursor[0]) << 8) | cursor[1]);
    cursor += 2;
    return result;
}

/**
 * @brief       按大端字节序读取 32 位无符号整数。
 * @param       cursor: 读取位置；读取后向前移动 4 字节。
 * @retval      读取到的 32 位无符号整数。
 */
std::uint32_t read_u32_be(const std::uint8_t *&cursor) noexcept
{
    const auto result = (static_cast<std::uint32_t>(cursor[0]) << 24) |
                        (static_cast<std::uint32_t>(cursor[1]) << 16) |
                        (static_cast<std::uint32_t>(cursor[2]) << 8) |
                        static_cast<std::uint32_t>(cursor[3]);
    cursor += 4;
    return result;
}

} // 匿名命名空间

/**
 * @brief       计算字节序列的 CRC16 校验值。
 * @param       bytes: 参与校验的字节序列。
 * @note        初始值为 0xFFFF，计算时使用反射多项式 0xA001。
 * @retval      计算得到的 CRC16 校验值。
 */
std::uint16_t crc16(std::span<const std::uint8_t> bytes) noexcept
{
    std::uint16_t crc = 0xFFFF;
    for (const auto byte : bytes)
    {
        crc ^= byte;
        for (int bit = 0; bit < 8; ++bit)
        {
            const bool low_bit_set = (crc & 1U) != 0;
            crc >>= 1;
            if (low_bit_set)
            {
                crc ^= 0xA001;
            }
        }
    }
    return crc;
}

/**
 * @brief       计算编码后完整帧所需的字节数。
 * @param       fields: 待编码的字段视图。
 * @retval      有效时返回完整帧长度；数据区超过发送上限时返回 std::nullopt。
 */
std::optional<std::size_t> encoded_frame_size(const FieldSpans &fields) noexcept
{
    //直接出发送payload的大小
    const auto length = tx_payload_size(counts_of(fields));

    if (!length)
    {
        return std::nullopt;
    }
    return *length + kFrameOverhead;
}

/**
 * @brief       将字段编码为包含帧头、数据区、CRC16 和帧尾的完整帧。
 * @param       command: 帧中的命令字节。
 * @param       fields: 待编码的字段视图。
 * @param       output: 用于保存完整帧的输出缓冲区。
 * @note        output 的长度必须等于 encoded_frame_size(fields) 的结果。
 * @retval      Status::ok: 编码成功；否则返回数据区过大或帧长度不匹配状态。
 */
Status encode(std::uint8_t command, const FieldSpans &fields,
              std::span<std::uint8_t> output) noexcept
{
    //获取每个类型的变量的数量
    const auto counts = counts_of(fields);

    //把数量变成发送payload的大小，并检查是否溢出
    const auto length = tx_payload_size(counts);

    //这里因为只有tx_payload_size()超过上限才会返回std::nullopt
    if (!length)
    {
        return Status::payload_too_large;
    }

    //这里不是指针
    if (output.size() != *length + kFrameOverhead)
    {
        return Status::frame_size_mismatch;
    }

    output[0] = 0xA5;
    output[1] = 0x5A;
    output[2] = static_cast<std::uint8_t>(*length);
    output[3] = command;

    //指针指到output[4]
    auto *cursor = output.data() + 4;

    for (std::size_t base = 0; base < counts.bools; base += 8) //bools = 20,base = 0
    {
        std::uint8_t packed = 0;

        for (std::size_t bit = 0; bit < 8 && base + bit < counts.bools; ++bit)
        {
            if (fields.bools[base + bit])
            {
                packed |= static_cast<std::uint8_t>(1U << bit);
            }
        }
        *cursor++ = packed;
    }
    for (const auto value : fields.int8s)
    {
        *cursor++ = std::bit_cast<std::uint8_t>(value);
    }
    for (const auto value : fields.int16s)
    {
        write_u16_be(std::bit_cast<std::uint16_t>(value), cursor);
    }
    for (const auto value : fields.int32s)
    {
        write_u32_be(std::bit_cast<std::uint32_t>(value), cursor);
    }
    for (const auto value : fields.float32s)
    {
        write_u32_be(std::bit_cast<std::uint32_t>(value), cursor);
    }

    const auto checksum = crc16(output.subspan(4, *length));
    write_u16_be(checksum, cursor);
    *cursor = 0xFF;
    return Status::ok;
}


//解析器FrameParser


/**
 * @brief       清空解析器当前缓存的数据。
 * @retval      无。
 */
void FrameParser::reset() noexcept 
{ 
    buffered_size_ = 0; 
}

/**
 * @brief       从解析器缓存中丢弃指定数量的前导字节。
 * @param       count: 要丢弃的字节数。
 * @note        count 不小于缓存长度时，缓存会被清空。
 * @retval      无。
 */
void FrameParser::discard_prefix(std::size_t count) noexcept
{
    if (count >= buffered_size_)
    {
        buffered_size_ = 0;
        return;
    }
    buffered_size_ -= count;
    std::memmove(buffer_.data(), buffer_.data() + count, buffered_size_);
}

/**
 * @brief       向解析器缓存追加一个字节。
 * @param       byte: 要追加的字节。
 * @note        缓存已满时先丢弃最早的一个字节。
 * @retval      无。
 */
void FrameParser::append(std::uint8_t byte) noexcept
{
    if (buffered_size_ == buffer_.size())
    {
        discard_prefix(1);
    }
    buffer_[buffered_size_++] = byte;
}

/**
 * @brief       从缓存中查找并取出下一帧有效数据。
 * @param       frame: 成功解析时接收命令和数据区的输出帧。
 * @note        无效帧头、超长数据区、CRC 错误或帧尾错误会触发逐字节重新同步。
 * @retval      true: 已取得完整有效帧；false: 当前缓存中没有完整有效帧。
 */
bool FrameParser::next(Frame &frame) noexcept
{
    while (buffered_size_ > 0)
    {
        if (buffer_[0] != 0xA5)
        {
            discard_prefix(1);
            continue;
        }
        if (buffered_size_ < 2)
        {
            return false;
        }
        if (buffer_[1] != 0x5A)
        {
            discard_prefix(1);
            continue;
        }
        if (buffered_size_ < 3)
        {
            return false;
        }

        const std::size_t length = buffer_[2];
        if (length > kMaxRxPayloadSize)
        {
            discard_prefix(1);
            continue;
        }
        const auto frame_size = length + kFrameOverhead;
        if (buffered_size_ < frame_size)
        {
            return false;
        }

        const auto received_crc = static_cast<std::uint16_t>(
            (static_cast<std::uint16_t>(buffer_[length + 4]) << 8) | buffer_[length + 5]);
        const auto expected_crc = crc16({buffer_.data() + 4, length});
        if (buffer_[length + 6] != 0xFF || received_crc != expected_crc)
        {
            discard_prefix(1);
            continue;
        }

        frame.command = buffer_[3];
        frame.payload_size = length;
        std::memcpy(frame.payload.data(), buffer_.data() + 4, length);
        discard_prefix(frame_size);
        return true;
    }
    return false;
}

/**
 * @brief       按指定字段数量解码数据区。
 * @param       payload: 待解码的数据区字节。
 * @param       counts: 各类型字段的预期数量。
 * @param       output: 解码成功时接收各类型字段的输出结构。
 * @note        数据区不携带各类型字段数量，调用方需要提供正确的 counts。
 * @retval      Status::ok: 解码成功；否则返回字段数量无效或数据区长度不匹配状态。
 */
Status decode(std::span<const std::uint8_t> payload, FieldCounts counts,
              DecodedFields &output) noexcept
{
    if (!counts_valid(counts))
    {
        return Status::invalid_counts;
    }
    if (payload.size() != payload_size(counts))
    {
        return Status::payload_size_mismatch;
    }

    DecodedFields decoded{};
    decoded.counts = counts;
    const auto *cursor = payload.data();

    for (std::size_t index = 0; index < counts.bools; ++index)
    {
        decoded.bools[index] = (cursor[index / 8] & (1U << (index % 8))) != 0;
    }
    cursor += (counts.bools + 7) / 8;

    for (std::size_t index = 0; index < counts.int8s; ++index)
    {
        decoded.int8s[index] = std::bit_cast<std::int8_t>(*cursor++);
    }
    for (std::size_t index = 0; index < counts.int16s; ++index)
    {
        decoded.int16s[index] = std::bit_cast<std::int16_t>(read_u16_be(cursor));
    }
    for (std::size_t index = 0; index < counts.int32s; ++index)
    {
        decoded.int32s[index] = std::bit_cast<std::int32_t>(read_u32_be(cursor));
    }
    for (std::size_t index = 0; index < counts.float32s; ++index)
    {
        decoded.float32s[index] = std::bit_cast<float>(read_u32_be(cursor));
    }

    output = decoded;
    return Status::ok;
}

} // wire_protocol 命名空间
