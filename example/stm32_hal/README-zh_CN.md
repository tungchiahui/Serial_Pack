# STM32HAL CMake C++ 模板

中文 | [English](README.md)

此目录用于接入 STM32CubeMX 生成的 CMake 工程。`applications` 可单独复制；需要板级延时实现时再复制 `bsp`。

## 接入方法

1. 在 CubeMX 生成的根目录 `CMakeLists.txt` 中，保留原有的 C 标准设置，并在 `project()` 前增加 C++ 标准设置：

```cmake
# Setup compiler settings
set(CMAKE_C_STANDARD 11 CACHE STRING "C language standard")
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 20 CACHE STRING "C++ language standard")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
```

`CACHE STRING` 把标准值保存为可覆盖的构建配置；已有构建目录中的缓存值不会因这里的默认值改变而自动更新。`*_EXTENSIONS ON` 允许使用编译器扩展，GCC 通常会选择 `gnu11` / `gnu++20` 方言；设为 `OFF` 则请求 `c11` / `c++20` 方言。CubeMX 生成的 C 代码建议保留 `C_EXTENSIONS ON`；自行编写的 C++ 若不使用 GNU 专有语法，可以将 `CXX_EXTENSIONS` 设为 `OFF`。

2. 复制 `cmake/user` , `applications` 和 `bsp`，**必须在顶层 `CMakeLists.txt` 中加入 `cmake/user`**，建议放在 CubeMX 子目录之后：

```cmake
# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add USER generated sources
add_subdirectory(cmake/user)
```

当前生成的 `project(${CMAKE_PROJECT_NAME})` 没有限定语言，CMake 默认启用 C 和 C++；`enable_language(C ASM)` 不会关闭 C++，所以这一行**不用改**。如果以后生成的 `project()` 显式写了 `LANGUAGES C ASM`，则需把 `CXX` 加进去。

3. 每新增一个 `.c` 或 `.cpp` 文件，手动把路径写进 `cmake/user/CMakeLists.txt` 的源文件列表。`.h`、`.hpp` 不需要列为编译源文件。

4. 在 CubeMX 的 `USER CODE` 区按项目需要调用 `cpp_main()`；`cpp_interface.h` 是供 C 文件调用的接口。当前 `cpp_main()` 保留原有的 `isRTOS` 行为，使用 FreeRTOS 时须自行设置该宏并选择合适的调用位置。

5. `bsp_delay` 使用 Cortex-M3/M4 的 DWT 周期计数器。`Init(sysclk)` 的参数单位是 MHz，应在系统时钟配置完成后调用。延时是忙等待，不会改写 SysTick 或 HAL 的 `HAL_Delay`。

## 清理脚本

`clean.bash` 始终针对脚本所在目录，按白名单清理文件。直接运行只预览；确认清单后运行 `./clean.bash --apply` 才会删除。它会删除白名单之外的顶层文件，包括 CubeMX 生成的 `Core`、`Drivers`、`CMakeLists.txt`、构建目录及 IDE 设置；保留 `applications`、`bsp`、`cmake/user` 和根目录的 `.ioc` 文件。

作者：Tung Chia-hui · [tungchiahui.cn](https://www.tungchiahui.cn) · 预发布版本
