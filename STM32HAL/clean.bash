#!/bin/bash

set -euo pipefail

echo "开始按白名单清理工程目录: $(pwd)"

# 先处理 cmake：只保留 cmake/user
if [ -d "cmake" ]; then
    find "cmake" -mindepth 1 -maxdepth 1 ! -name "user" -exec rm -rf -- {} +
fi

# 清理工程根目录：仅保留白名单项
find . -mindepth 1 -maxdepth 1 \
    ! -name ".git" \
    ! -name "clean.bash" \
    ! -name "cmake" \
    ! -name "applications" \
    ! -name "bsp" \
    ! -name "README.md" \
    ! -name "README-zh_CN.md" \
    ! -name "LICENSE" \
    ! -name "*.ioc" \
    -exec rm -rf -- {} +

echo "清理完成。"
echo "保留内容："
echo "  - cmake/user"
echo "  - applications"
echo "  - bsp"
echo "  - README.md 和 README-zh_CN.md"
echo "  - LICENSE"
echo "  - 根目录 *.ioc 文件"
