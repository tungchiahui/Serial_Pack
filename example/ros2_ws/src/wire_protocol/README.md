# wire_protocol

纯 C++20、无 ROS 依赖的定长串口协议。主 API 只有：

```text
wire_protocol::dispatch(command, handler)
wire_protocol::make_protocol(dispatch...)
protocol.pack(command, fields...)
protocol.feed(bytes)
protocol.reset()
```

handler 的参数类型就是该命令的 payload schema。协议从签名推导字段数量与类型；
业务层不必接触 Parser、Frame、payload、字段计数或手动解包。

## 基本用法

```cpp
#include <wire_protocol/protocol.hpp>

void handle_cmd_vel(std::uint32_t seq, float vx, float vy, float wz)
{
    // 处理速度命令
}

void handle_mode(std::uint32_t seq, std::int32_t mode)
{
    // 处理模式命令
}

auto protocol = wire_protocol::make_protocol(
    wire_protocol::dispatch(0x01, handle_cmd_vel),
    wire_protocol::dispatch(0x02, handle_mode));

// 发送：返回拥有字节的定长 std::array，长度在编译期确定。
auto tx = protocol.pack(0x01, seq, vx, vy, wz);
serial_driver.async_write(tx); // 当前 SerialTransport 会复制发送数据

// 串口接收回调中：bytes 是本次收到的 std::span<const uint8_t>。
protocol.feed(bytes);

// 串口异常、断线、超时或重新连接时丢弃残留半帧。
protocol.reset();
```

`protocol` 应与串口连接一样长期存在；每路串口使用独立实例。接收回调不能重入同一个
`protocol.feed()`，也不要在多个线程或中断中并发调用同一实例。回调应正常返回。
一个 `feed()` 可包含半包、粘包和噪声。未注册命令、长度不符合 handler schema 的帧会被忽略；
同一命令注册多次时仅调用第一个。handler 必须在 Protocol 生命周期内保持有效，
例如成员函数绑定的对象不能先于 Protocol 销毁。

## 类成员函数与 lambda

成员函数按 `dispatch(command, &Class::method, object_pointer)` 注册，支持普通、`const`、
`noexcept` 和 `const noexcept` 成员函数。捕获 lambda 可直接保存，无类型擦除：

```cpp
class SerialComm
{
  public:
    SerialComm()
        : protocol_(wire_protocol::make_protocol(
              wire_protocol::dispatch(0x01, &SerialComm::handle_cmd_vel, this),
              wire_protocol::dispatch(0x02, &SerialComm::handle_mode, this)))
    {
    }

    void on_bytes(std::span<const std::uint8_t> bytes)
    {
        protocol_.feed(bytes);
    }

  private:
    void handle_cmd_vel(std::uint32_t, float, float, float);
    void handle_mode(std::uint32_t, std::int32_t);

    // 成员变量不能用 auto；在未求值的 decltype 中传一个同类型空指针即可。
    using ProtocolType = decltype(wire_protocol::make_protocol(
        wire_protocol::dispatch(0x01, &SerialComm::handle_cmd_vel,
                                static_cast<SerialComm*>(nullptr)),
        wire_protocol::dispatch(0x02, &SerialComm::handle_mode,
                                static_cast<SerialComm*>(nullptr))));
    ProtocolType protocol_;
};
```

如果仅在函数内部使用，也可以直接注册捕获 lambda：

```cpp
int calls = 0;
auto protocol = wire_protocol::make_protocol(
    wire_protocol::dispatch(0x03,
        [&calls](std::uint32_t seq, float value)
        {
            ++calls;
            // 处理 seq 和 value
        }));
```

handler 必须返回 `void`，payload 参数按值传递且有明确类型。
支持普通函数、普通函数指针、明确参数类型的 lambda/functor 和上述成员函数；
暂不支持 generic lambda (`auto` 参数)、`std::bind`、引用参数、指针参数或结构体自动序列化。
`dispatch` 中的 callable 存储在 `Protocol` 的 `std::tuple` 中，编译期确定，无动态注册。

## 线格式

```text
A5 5A | LEN(1) | CMD(1) | DATA(LEN) | CRC16(2) | FF
```

CRC16 初值 `0xFFFF`、反射多项式 `0xA001`；**覆盖 LEN + CMD + DATA**，
不包含帧头、CRC 本身和帧尾。CRC 发送时高字节在前。此覆盖范围与第一版
“只覆盖 DATA”不同，两端需要同时升级；旧版完整帧不能直接与本版互通。
它也不是 Modbus RTU：Modbus 的帧结构与 CRC 字节发送顺序不同。

字段按类型分组，依次为 `bool`、8 位整数、16 位整数、32 位整数、`float32`；
同组内保持参数顺序（包括展开固定数组的顺序），不是全局参数顺序。
`bool` 每 8 个占 1 字节，第一个放在 bit 0。整数和浮点位模式大端。
有符号与无符号整数在同一宽度组。当前支持：

```text
bool
int8_t / uint8_t
int16_t / uint16_t
int32_t / uint32_t
float
std::array<上述类型, N>
```

`double`、64 位整数、枚举、字符串、动态数组等在编译期拒绝；浮点常量请写成 `1.0f`。
发送 DATA 最多 100 字节，超限编译失败；接收解析器的内部缓存也按 100 字节
DATA 配置。线上不携带字段类型信息：长度相同但 schema 不同的错误无法自动识别，
发送端与接收端必须约定每条命令的字段类型和各组内顺序。

解析器按字节流工作，支持半包、粘包、前导噪声和 CRC／帧尾错误后的重新同步。
若损坏的 LEN 仍在合法范围内，解析器可能暂时等待更多字节；连接重建或接收超时
应调用 `reset()`。CRC 是传输错误检测，不提供身份认证。

## STM32 与构建

协议由 `include/wire_protocol/protocol.hpp` 和 `src/protocol.cpp` 组成，
不使用 `new`、`std::function`、`std::vector`、异常、RTTI 或虚函数。
`std::array` 和 `std::tuple` 均为对象内固定存储；`std::span` 只借用接收数据。
STM32 工具链需提供 C++20 的 `std::span`、`std::bit_cast`、concepts 和 `std::invoke`。
通常在主循环／任务中喂入 DMA 接收片段；如直接把 `pack()` 返回数组交给 UART DMA，
数组必须活到 DMA 发送完成。

```sh
cmake -S src/wire_protocol -B /tmp/wire_protocol-build
cmake --build /tmp/wire_protocol-build
```

STM32 工程需要同时加入头文件和 `src/protocol.cpp`。CMake 工程可通过
`add_subdirectory(...)`、
`find_package(wire_protocol CONFIG REQUIRED)` 引入并链接
`wire_protocol::wire_protocol`。`package.xml` 只供 colcon 发现普通 CMake 包，
构建本身不要求 ament。
