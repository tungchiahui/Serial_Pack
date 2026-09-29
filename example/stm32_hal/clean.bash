#!/bin/bash

set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

case "${1:---dry-run}" in
    --dry-run) mode=preview ;;
    --apply) mode=apply ;;
    -h|--help)
        echo "用法：$0 [--dry-run|--apply]"
        echo "默认只预览；传入 --apply 才会删除脚本所在目录中的文件。"
        exit 0
        ;;
    *)
        echo "未知参数：$1" >&2
        exit 2
        ;;
esac

if [ "$#" -gt 1 ]; then
    echo "只能指定一个参数。" >&2
    exit 2
fi

# 防止脚本被误放在其他目录后执行。
if [ "$project_dir" = / ] || [ ! -d "$project_dir/applications" ] || [ ! -d "$project_dir/cmake/user" ]; then
    echo "拒绝清理：脚本所在目录缺少 applications 或 cmake/user。" >&2
    exit 1
fi

echo "工程目录：$project_dir"

# 收集待删除路径：cmake 下保留 user，根目录按白名单保留。
# .clangd 不在白名单中，会随 CubeMX 生成文件一起删除。
mapfile -d '' -t targets < <(
    find "$project_dir/cmake" -mindepth 1 -maxdepth 1 ! -name "user" -print0
    find "$project_dir" -mindepth 1 -maxdepth 1 \
        ! -name ".git" \
        ! -name "clean.bash" \
        ! -name "cmake" \
        ! -name "applications" \
        ! -name "bsp" \
        ! -name "README.md" \
        ! -name "README-zh_CN.md" \
        ! -name "LICENSE" \
        ! -name "*.ioc" \
        -print0
)

if [ "${#targets[@]}" -eq 0 ]; then
    echo "没有需要清理的文件。"
    exit 0
fi

echo "待清理路径："
for target in "${targets[@]}"; do
    printf '  %q\n' "${target#"$project_dir"/}"
done

if [ "$mode" = preview ]; then
    echo "以上仅为预览；执行 $0 --apply 才会删除。"
    exit 0
fi

for target in "${targets[@]}"; do
    rm -rf -- "$target"
done

echo "清理完成。"
echo "保留内容："
echo "  - cmake/user"
echo "  - applications"
echo "  - bsp"
echo "  - README.md 和 README-zh_CN.md"
echo "  - LICENSE"
echo "  - 根目录 *.ioc 文件"
