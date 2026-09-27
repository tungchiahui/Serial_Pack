#ifndef WIRE_PROTOCOL_PROTOCOL_HPP_
#define WIRE_PROTOCOL_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace wire_protocol
{

// 帧格式：A5 5A、数据区长度、命令、数据区、CRC16、FF。
// CRC16 仅校验数据区，发送时高字节在前。

inline constexpr std::size_t kFrameOverhead = 7;

// 发送数据区的最大长度。
inline constexpr std::size_t kMaxTxPayloadSize = 100;

// 接收端各类型字段的数量上限。
inline constexpr std::size_t kMaxBoolCount = 32;
inline constexpr std::size_t kMaxInt8Count = 10;
inline constexpr std::size_t kMaxInt16Count = 10;
inline constexpr std::size_t kMaxInt32Count = 10;
inline constexpr std::size_t kMaxFloat32Count = 10;
inline constexpr std::size_t kMaxRxPayloadSize = (kMaxBoolCount + 7) / 8 + kMaxInt8Count +
                                                 2 * kMaxInt16Count + 4 * kMaxInt32Count +
                                                 4 * kMaxFloat32Count;

struct FieldCounts
{
    std::size_t bools{};
    std::size_t int8s{};
    std::size_t int16s{};
    std::size_t int32s{};
    std::size_t float32s{};
};

// 序列化时各类型字段始终按 bool、int8、int16、int32、float32 的顺序排列。
struct FieldSpans
{
    std::span<const bool> bools{};
    std::span<const std::int8_t> int8s{};
    std::span<const std::int16_t> int16s{};
    std::span<const std::int32_t> int32s{};
    std::span<const float> float32s{};
};

enum class Status
{
    ok,
    invalid_counts,
    payload_too_large,
    frame_size_mismatch,
    payload_size_mismatch,
};

/**
 * @brief       计算编码后的完整帧长度。
 * @param       fields: 待编码的字段视图。
 * @note        数据区最多为 100 字节；计算过程不分配内存。
 * @retval      有效时返回完整帧长度；数据区超过发送上限时返回 std::nullopt。
 */
std::optional<std::size_t> encoded_frame_size(const FieldSpans &fields) noexcept;

/**
 * @brief       将字段编码为包含帧头、数据区、CRC16 和帧尾的完整帧。
 * @param       command: 帧中的命令字节。
 * @param       fields: 待编码的字段视图。
 * @param       output: 用于保存完整帧的输出缓冲区。
 * @note        output 的长度必须等于 encoded_frame_size(fields) 的结果。
 * @retval      Status::ok: 编码成功；否则返回数据区过大或帧长度不匹配状态。
 */
Status encode(std::uint8_t command, const FieldSpans &fields,
              std::span<std::uint8_t> output) noexcept;

struct Frame
{
    std::uint8_t command{};
    std::array<std::uint8_t, kMaxRxPayloadSize> payload{};
    std::size_t payload_size{};

    /**
     * @brief       取得帧中有效数据区的只读视图。
     * @retval      长度为 payload_size 的数据区字节视图。
     */
    std::span<const std::uint8_t> payload_bytes() const noexcept
    {
        return {payload.data(), payload_size};
    }
};

// 每路串口输入应使用独立的解析器。
class FrameParser
{
  public:
    /**
     * @brief       输入任意长度的字节块，并逐帧回调有效数据。
     * @param       input: 待解析的输入字节。
     * @param       on_frame: 接收已通过 CRC16 和帧尾检查的数据帧的回调。
     * @note        回调收到的 Frame 仅在本次调用期间有效；需要长期保存时应复制。
     * @retval      无。
     */
    template <typename OnFrame> void feed(std::span<const std::uint8_t> input, OnFrame &&on_frame)
    {
        Frame frame{};
        for (const auto byte : input)
        {
            append(byte);
            while (next(frame))
            {
                on_frame(frame);
            }
        }
    }

    /**
     * @brief       清空解析器当前缓存的数据。
     * @retval      无。
     */
    void reset() noexcept;

  private:
    /**
     * @brief       向解析器缓存追加一个字节。
     * @param       byte: 要追加的字节。
     * @note        缓存已满时先丢弃最早的一个字节。
     * @retval      无。
     */
    void append(std::uint8_t byte) noexcept;

    /**
     * @brief       从缓存中查找并取出下一帧有效数据。
     * @param       frame: 成功解析时接收命令和数据区的输出帧。
     * @retval      true: 已取得完整有效帧；false: 当前缓存中没有完整有效帧。
     */
    bool next(Frame &frame) noexcept;

    /**
     * @brief       从解析器缓存中丢弃指定数量的前导字节。
     * @param       count: 要丢弃的字节数。
     * @note        count 不小于缓存长度时，缓存会被清空。
     * @retval      无。
     */
    void discard_prefix(std::size_t count) noexcept;

    std::array<std::uint8_t, kMaxRxPayloadSize + kFrameOverhead> buffer_{};
    std::size_t buffered_size_{};
};

struct DecodedFields
{
    FieldCounts counts{};
    std::array<bool, kMaxBoolCount> bools{};
    std::array<std::int8_t, kMaxInt8Count> int8s{};
    std::array<std::int16_t, kMaxInt16Count> int16s{};
    std::array<std::int32_t, kMaxInt32Count> int32s{};
    std::array<float, kMaxFloat32Count> float32s{};
};

/**
 * @brief       按指定字段数量解码数据区。
 * @param       payload: 待解码的数据区字节。
 * @param       counts: 各类型字段的预期数量。
 * @param       output: 解码成功时接收各类型字段的输出结构。
 * @note        帧只携带命令和数据区总长度；各类型字段数量由调用方提供。
 * @retval      Status::ok: 解码成功；否则返回字段数量无效或数据区长度不匹配状态。
 */
Status decode(std::span<const std::uint8_t> payload, FieldCounts counts,
              DecodedFields &output) noexcept;

/**
 * @brief       计算字节序列的 CRC16 校验值。
 * @param       bytes: 参与校验的字节序列。
 * @note        初始值为 0xFFFF，计算时使用反射多项式 0xA001。
 * @retval      计算得到的 CRC16 校验值。
 */
std::uint16_t crc16(std::span<const std::uint8_t> bytes) noexcept;

} // wire_protocol 命名空间

#endif // WIRE_PROTOCOL_PROTOCOL_HPP_
