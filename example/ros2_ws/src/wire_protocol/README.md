# wire_protocol（第一版）

独立 C++20 头文件库，无 ROS、Asio、HAL 依赖。协议代码不使用堆内存、
异常、RTTI、虚函数或 `std::function`。帧数组和解析器缓存均为定长 `std::array`；
`std::span` 只借用内存，不分配。支持关闭异常和 RTTI 的编译。

## 发送和接收

```cpp
#include <wire_protocol/protocol.hpp>

// uint32_t 直接传，不再 bit_cast，也不用 FieldCounts/FieldSpans。
auto tx = wire_protocol::pack(0x01, seq, vx, vy, wz);
serial_driver.async_write(tx);  // 现有 SerialTransport 接口直接兼容

// 放在类成员中，每路串口一个，不要每次收到数据都重新创建。
wire_protocol::Parser<> parser;

// 串口接收回调中：
parser.feed(bytes, [&](const wire_protocol::Frame& frame) {
    switch (frame.command) {
    case 0x01: {
        std::uint32_t seq{};
        float vx{}, vy{}, wz{};
        if (!frame.unpack(seq, vx, vy, wz)) return; // 长度不符，变量保持原值
        // 在这里处理速度命令
        break;
    }
    case 0x02: {
        std::uint32_t seq{};
        std::int32_t mode{};
        if (!frame.unpack(seq, mode)) return;
        // 在这里处理模式；打印 mode 用 %d
        break;
    }
    }
});
```

发送模式命令：`auto tx = wire_protocol::pack(0x02, seq, mode);`。
固定数组也可直接传：`pack(0x03, std::array<bool, 9>{...}, std::array<float, 3>{...})`，
接收时提供同类型、同长度的数组。支持空载荷 `pack(0x04)` / `frame.unpack()`。

## 线格式兼容

`A5 5A | payload长度(1) | command(1) | payload | CRC16(2) | FF`

保留旧版规则：bool 紧凑打包（第一个为 bit0），然后依次为 8 位整数、
16 位整数、32 位整数、float32。每组内按参数顺序排列，数组按下标展开。
有符号和无符号整数在同一个宽度组内；多字节数值和 CRC 均大端。
CRC16 初值 `0xFFFF`，反射多项式 `0xA001`，只校验 payload。

**参数会按类型分组，不是直接按参数顺序上线路。** 例如 `pack(cmd, float值, seq)`
仍然先写 seq 再写 float。两端需约定同一布局；线上不携带类型信息，
`unpack` 只能检查总长度，无法发现所有类型/数量约定错误。
不序列化 C++ 结构体内存，避免 padding、对齐和主机字节序问题。

支持 bool、int8/uint8、int16/uint16、int32/uint32、float 及其一维 `std::array`。
`double`、64 位整数、枚举、指针、动态数组等会在编译期拒绝；浮点参数使用 `1.0f`。
发送 payload 上限保持 100 字节，超限编译失败。接收默认同样为 100 字节，
旧版接收最大 114 字节需要用 `Parser<114>`；可配置至 255 字节。
新版本取消每种类型单独的接收数量上限。

解析器支持半包、粘包、前导噪声、CRC/尾字节错误后的逐字节重新同步。
长度字节损坏为合法的大长度时，需要等候足够字节才能判断该候选帧是否有效；
串口断开、接收超时或 MCU UART 出错时由上层调用 `parser.reset()`。
CRC 不保护 command 和长度，这是沿用旧协议的限制。
`Frame` 和 payload 只在回调期间有效，复制 Frame 也只是复制视图；需要保留数据时
应立即 `unpack` 到自己的变量/数组。不要在回调里重入同一个解析器，
不要从多个线程/中断同时操作同一个解析器；回调应正常返回。

## 构建与 STM32

可以直接拷贝 `include/wire_protocol`，开启 C++20 后包含头文件。
STM32 工程需要工具链标准库支持 `std::span`、`std::bit_cast`、concepts；
库不初始化 UART，也不依赖操作系统。通常在主循环/任务中喂入 DMA 接收片段。
发送数组需存活至 UART DMA 发送完成（HAL 不会替你复制数组）。

```sh
cmake -S src/wire_protocol -B /tmp/wire_protocol-build \
  -DWIRE_PROTOCOL_BUILD_TESTS=ON -DWIRE_PROTOCOL_BUILD_EXAMPLES=ON
cmake --build /tmp/wire_protocol-build
ctest --test-dir /tmp/wire_protocol-build --output-on-failure
```

上层工程可 `add_subdirectory(...)` 或安装后 `find_package(wire_protocol CONFIG REQUIRED)`，
并 `target_link_libraries(app PRIVATE wire_protocol::wire_protocol)`。
`package.xml` 仅供 colcon 发现普通 cmake 包，构建不要求 ament。

本版附独立示例，现有 serial_comm 接入时只需加 CMake 链接/包依赖，
把模拟原始字节的发送改为 pack、接收改为 Parser::feed。
