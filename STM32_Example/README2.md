# STM32 HAL 项目构建指南

## 项目概述
本项目基于 STM32 HAL 库开发，采用混合编程模式（C++ 主逻辑 + C 硬件抽象层）。请按照以下说明配置开发环境。

---

## 前置要求

- **STM32CubeMX** (v6.0+)
- **ARM GCC 工具链**
- **IDE/编辑器** (VSCode, CLion, STM32CubeIDE 等)

---

## 项目设置步骤

### 1. 克隆仓库

```bash
git clone <repository-url>
cd <project-directory>
```

### 2. 备份现有 main.c 文件（如果存在）

```bash
# 如果项目中有 main.c 文件，先备份
cp Core/Src/main.c Core/Src/main.c.backup
```

### 3. 使用 STM32CubeMX 生成 HAL 库

- 打开 STM32CubeMX
- 选择 **File → Load Project** 打开项目中的 `.ioc` 文件
- 检查配置并根据需要调整外设设置
- 点击 **GENERATE CODE** 重新生成 HAL 库代码

### 4. 处理 main.c 文件

#### 情况 A：如果您使用 C++ 开发

在 `main.c` 中的 `main()` 函数内，找到 `while(1)` 循环，在其上方添加对 C++ 入口函数的调用：

```c
/* 在初始化代码后，主循环前添加 */
cpp_main();  // C++ 入口函数

/* Infinite loop */
while (1)
{
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
}
```

**如果使用 RTOS（如 FreeRTOS）：**

```c
/* 在 RTOS 初始化前调用 C++ 入口函数 */
cpp_main();  // C++ 入口函数

/* Init FreeRTOS */
osKernelInitialize();
```

#### 情况 B：如果您只使用 C 语言开发

```bash
# 恢复您备份的 main.c 文件
mv Core/Src/main.c.backup Core/Src/main.c
```

### 5. 构建项目

**使用 CMake（推荐，或者你使用KEIL MDK5和MDK6也可以）**

```bash
mkdir build
cd build
cmake ..
make
```

**使用 Makefile（如果提供）**

```bash
make all
```

---

## 项目结构说明

```
├── Core/
│   ├── Inc/          # 头文件目录
│   ├── Src/          # 源文件目录
│   └── Startup/      # 启动文件
├── Drivers/          # HAL 库（由 CubeMX 生成，不上传至 Git）
├── .gitignore        # Git 忽略规则
├── .ioc              # CubeMX 配置文件
└── README.md         # 本文件
```

---

## 注意事项

- **HAL 库管理**：本项目不直接包含 HAL 库文件，需通过 CubeMX 生成
- **C++ 支持**：确保在 CubeMX 中启用 C++ 支持（Project Manager → Advanced Settings）
- **编译选项**：如果使用 C++，需要在链接器中添加 C++ 标准库（如 `-lstdc++`）

---

## 故障排除

- **编译错误**：确保已正确安装 ARM GCC 工具链
- **链接错误**：检查是否缺少 C++ 库链接标志
- **CubeMX 生成问题**：验证 `.ioc` 文件版本与 CubeMX 版本兼容

---

## 协作指南

- 修改外设配置时，请通过 CubeMX 调整并重新生成代码
- 重要功能修改请通过 Issue 讨论
- 提交代码前请测试基本功能正常运行

