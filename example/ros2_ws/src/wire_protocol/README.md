# wire_protocol 使用说明

`wire_protocol` 是一个 ROS 2 C++ 库，负责将字段编码成协议帧，并从收到的字节流中提取、校验和解码协议帧。串口的打开、读写和重连由调用方负责。

## 构建与引用

本包要求 C++23。在工作区根目录执行：

```bash
colcon build --packages-select wire_protocol
```

在 C++ 文件中引入：

```cpp
#include "wire_protocol/protocol.hpp"
```

其他 ROS 2 包使用本库时，在 `package.xml` 中声明 `<depend>wire_protocol</depend>`，并在 CMake 中使用 `find_package(wire_protocol REQUIRED)` 和 `ament_target_dependencies(你的目标 wire_protocol)`。

## 帧格式

设数据区长度为 `N` 字节，完整帧长度为 `N + 7` 字节：

| 位置 | 内容 | 说明 |
| --- | --- | --- |
| `0`、`1` | `0xA5`、`0x5A` | 帧头 |
| `2` | `N` | 数据区长度，1 字节 |
| `3` | `command` | 命令字节 |
| `4` 至 `3 + N` | 数据区 | 各类型字段按固定顺序排列 |
| `4 + N`、`5 + N` | CRC16 高字节、低字节 | 仅校验数据区 |
| `6 + N` | `0xFF` | 帧尾 |

数据区的类型顺序固定为 **`bool`、`int8`、`int16`、`int32`、`float32`**。布尔值每 8 个打包为 1 字节，第一个值位于最低位；不足 8 个时仍占 1 字节。16 位和 32 位数值按高字节在前写入，`float32` 按 IEEE 754 单精度的 32 位位模式写入。CRC16 的初始值为 `0xFFFF`，计算时使用反射多项式 `0xA001`。

**帧中没有各类型字段的数量。**接收方必须根据 `command` 等双方约定的规则，确定解码时使用的 `FieldCounts`。同一长度可能对应不同的字段组合，不能只凭长度判断类型。

发送数据区上限为 `100` 字节；接收解析器允许的数据区上限为 `114` 字节。解码时各类型字段还有独立上限：`bool` 32 个，其余四种类型各 10 个。若收发双方都使用本库，设计字段布局时应同时满足这些限制。

## 发送：构造字段并编码

下面以命令 `0x01` 为例，发送一个 32 位序号和三个 `float`。`seq` 是 `uint32_t`，而 `FieldSpans::int32s` 接收 `int32_t`；使用 `std::bit_cast` 可以保留序号的全部 32 位。

```cpp
#include "wire_protocol/protocol.hpp"

#include <array>
#include <bit>
#include <cstdint>
#include <span>
#include <vector>

std::vector<std::uint8_t> make_velocity_frame(std::uint32_t seq,
                                              float vx, float vy, float wz)
{
    const std::array<std::int32_t, 1> int32_values{
        std::bit_cast<std::int32_t>(seq)};
    const std::array<float, 3> float_values{vx, vy, wz};

    wire_protocol::FieldSpans fields{};
    fields.int32s = std::span<const std::int32_t>{int32_values};
    fields.float32s = std::span<const float>{float_values};

    const auto frame_size = wire_protocol::encoded_frame_size(fields);
    if (!frame_size) {
        return {}; // 数据区超过发送上限
    }

    std::vector<std::uint8_t> frame(*frame_size);
    if (wire_protocol::encode(0x01, fields, frame) != wire_protocol::Status::ok) {
        return {};
    }
    return frame;
}
```

`encode()` 要求输出缓冲区**恰好**具有 `encoded_frame_size()` 返回的长度。将空的 `std::vector<uint8_t>` 直接交给 `encode()` 会得到 `Status::frame_size_mismatch`。上例返回空向量表示编码失败，调用方不应将其加入发送队列。`FieldSpans` 中的 `span` 只引用原数据，不复制字段；执行 `encode()` 时，`int32_values` 和 `float_values` 必须仍然存在。

`std::vector<uint32_t>` 不能直接赋给 `fields.int32s`，因为 `span<const int32_t>` 要求元素类型匹配；`std::vector<uint8_t>` 也不能承载完整的 32 位序号。若要发送布尔数组，不要使用 `std::vector<bool>` 作为 `fields.bools` 的来源，它不提供普通的连续 `bool` 存储；可以使用 `std::array<bool, N>`。

## 接收：先提取帧，再解码字段

串口的一次读取可能只得到半帧，也可能同时得到多帧。每路输入保留一个长期存在的 `FrameParser`，并且只把本次实际收到的字节交给 `feed()`：

```cpp
// 放在接收类的成员中，不能在每次读取回调里重新创建。
wire_protocol::FrameParser parser_;
std::array<std::uint8_t, 1024> rx_buffer_{};

// 以下代码放在读取成功的回调中；bytes_transferred 是本次实际读取长度。
parser_.feed(
    std::span<const std::uint8_t>{rx_buffer_.data(), bytes_transferred},
    [this](const wire_protocol::Frame &frame) {
        if (frame.command != 0x01) {
            return; // 其他命令需要按各自约定的字段数量处理
        }

        // 此布局仅对应上面的 0x01 示例：1 个 int32、3 个 float32。
        const wire_protocol::FieldCounts counts{0, 0, 0, 1, 3};
        wire_protocol::DecodedFields decoded{};
        if (wire_protocol::decode(frame.payload_bytes(), counts, decoded)
            != wire_protocol::Status::ok) {
            return;
        }

        const auto seq = std::bit_cast<std::uint32_t>(decoded.int32s[0]);
        const float vx = decoded.float32s[0];
        const float vy = decoded.float32s[1];
        const float wz = decoded.float32s[2];
        // 在此处理 seq、vx、vy、wz。
    });

// 处理完本次数据后，再注册下一次异步读取。
start_async_read();
```

`feed()` 会处理拆分或合并的帧，并在帧头、CRC16 和帧尾检查通过后调用回调。`decode()` 只接收**数据区**，因此使用 `frame.payload_bytes()`，不要把整个 `rx_buffer_` 或完整帧传给它。回调参数 `frame` 是临时对象；如果需要在回调结束后使用，应复制数据。串口断开后若希望丢弃残留的半帧，可以调用 `parser_.reset()`。

## 主要接口与状态

| 接口 | 用途 |
| --- | --- |
| `encoded_frame_size(fields)` | 计算编码后的完整帧长度；数据区过大时返回 `std::nullopt` |
| `encode(command, fields, output)` | 把字段写入已按准确长度分配的输出缓冲区 |
| `FrameParser::feed(input, on_frame)` | 从任意分段的输入字节中提取有效帧 |
| `FrameParser::reset()` | 清除解析器内尚未组成完整帧的缓存 |
| `Frame::payload_bytes()` | 取得不含帧头、命令、CRC16 和帧尾的数据区视图 |
| `decode(payload, counts, output)` | 按指定字段数量解码数据区 |
| `crc16(bytes)` | 单独计算字节序列的 CRC16 |

| `Status` | 含义 |
| --- | --- |
| `ok` | 编码或解码成功 |
| `invalid_counts` | 解码时某类字段数量超出接收上限 |
| `payload_too_large` | 编码的数据区超过发送上限 |
| `frame_size_mismatch` | 编码输出缓冲区的长度不正确 |
| `payload_size_mismatch` | 解码数据区长度与 `FieldCounts` 不匹配 |

`decode()` 成功后才会把结果写入 `output`。解析器负责完整帧校验，`decode()` 本身不检查帧头、CRC16 或帧尾。
