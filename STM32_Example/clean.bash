#!/bin/bash

# 安全清理当前目录的CubeMX生成文件
echo "开始清理当前目录中的CubeMX生成文件..."

# 处理当前目录
echo "处理当前目录: $(pwd)"

# 删除库文件夹
rm -rf "Drivers" 2>/dev/null
rm -rf "Middlewares" 2>/dev/null
rm -rf "MDK-ARM" 2>/dev/null

# 清理Core/Src目录（保留main.c）
if [ -d "Core/Src" ]; then
    find "Core/Src" -type f ! -name "main.c" -delete
fi

# 清理Core/Inc目录（保留main.h）
if [ -d "Core/Inc" ]; then
    find "Core/Inc" -type f ! -name "main.h" -delete
fi

# 可选：删除CubeMX配置文件
# find . -maxdepth 1 -type f \( -name "*.ioc" -o -name "*.mxproject" \) -delete

echo "清理完成！"
echo "保留的文件："
echo "  - main.c 和 main.h"
echo "  - 项目核心代码文件"
echo "删除的文件："
echo "  - Drivers, Middlewares, MDK-ARM 文件夹"
echo "  - Core/Src 和 Core/Inc 中除 main.c/main.h 外的所有文件"
echo "  - CubeMX自动生成的配置文件"
