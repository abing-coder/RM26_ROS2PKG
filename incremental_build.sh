#!/bin/bash

# 增量编译脚本 - 只编译修改过的文件，并检测 YAML 配置文件变化

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"
CACHE_DIR="$WORKSPACE_DIR/.build_cache"
YAML_CACHE_FILE="$CACHE_DIR/yaml_checksums.txt"

echo "=========================================="
echo "开始增量编译..."
echo "工作空间: $WORKSPACE_DIR"
echo "=========================================="

cd "$WORKSPACE_DIR"

# 创建缓存目录
mkdir -p "$CACHE_DIR"

# 检测 YAML 配置文件变化的函数
check_yaml_changes() {
    echo "检查 YAML 配置文件变化..."

    # 查找所有 src 目录下的 YAML 配置文件
    YAML_FILES=$(find "$WORKSPACE_DIR/src" -type f \( -name "*.yaml" -o -name "*.yml" \) -path "*/config/*" 2>/dev/null)

    if [ -z "$YAML_FILES" ]; then
        echo "未找到 YAML 配置文件"
        return 0
    fi

    # 计算当前 YAML 文件的校验和
    CURRENT_CHECKSUMS=$(echo "$YAML_FILES" | xargs md5sum 2>/dev/null | sort)

    # 如果缓存文件不存在，创建它
    if [ ! -f "$YAML_CACHE_FILE" ]; then
        echo "首次运行，创建 YAML 校验和缓存..."
        echo "$CURRENT_CHECKSUMS" > "$YAML_CACHE_FILE"
        return 0
    fi

    # 读取上次的校验和
    PREVIOUS_CHECKSUMS=$(cat "$YAML_CACHE_FILE")

    # 比较校验和
    if [ "$CURRENT_CHECKSUMS" != "$PREVIOUS_CHECKSUMS" ]; then
        echo "=========================================="
        echo "检测到 YAML 配置文件变化！"
        echo "=========================================="

        # 找出哪些文件发生了变化
        CHANGED_PACKAGES=""
        while IFS= read -r yaml_file; do
            CURRENT_MD5=$(echo "$CURRENT_CHECKSUMS" | grep "$yaml_file" | awk '{print $1}')
            PREVIOUS_MD5=$(echo "$PREVIOUS_CHECKSUMS" | grep "$yaml_file" | awk '{print $1}')

            if [ "$CURRENT_MD5" != "$PREVIOUS_MD5" ]; then
                echo "  变化的文件: $yaml_file"

                # 提取包名（从 src/包名/... 路径中提取）
                PACKAGE_NAME=$(echo "$yaml_file" | sed -n 's|.*/src/\([^/]*\)/.*|\1|p')
                if [ -n "$PACKAGE_NAME" ] && [[ ! "$CHANGED_PACKAGES" =~ "$PACKAGE_NAME" ]]; then
                    CHANGED_PACKAGES="$CHANGED_PACKAGES $PACKAGE_NAME"
                fi
            fi
        done <<< "$YAML_FILES"

        # 更新缓存
        echo "$CURRENT_CHECKSUMS" > "$YAML_CACHE_FILE"

        # 返回需要重新编译的包列表
        echo "$CHANGED_PACKAGES"
        return 1
    else
        echo "YAML 配置文件无变化"
        return 0
    fi
}

# 检查 YAML 变化
PACKAGES_TO_REBUILD=$(check_yaml_changes)
YAML_CHANGED=$?

# 根据检测结果决定编译策略
if [ $YAML_CHANGED -eq 1 ] && [ -n "$PACKAGES_TO_REBUILD" ]; then
    echo "=========================================="
    echo "需要重新编译以下包以更新配置文件:"
    for pkg in $PACKAGES_TO_REBUILD; do
        echo "  - $pkg"
    done
    echo "=========================================="

    # 强制重新编译这些包（清理后重新编译）
    for pkg in $PACKAGES_TO_REBUILD; do
        echo "正在重新编译包: $pkg"
        colcon build --packages-select $pkg --cmake-clean-cache
        if [ $? -ne 0 ]; then
            echo "=========================================="
            echo "包 $pkg 编译失败，请检查错误信息"
            echo "=========================================="
            exit 1
        fi
    done

    echo "配置文件相关的包已重新编译完成"
    echo "=========================================="
fi

# 执行常规增量编译（编译其他修改的代码）
echo "执行常规增量编译..."
colcon build

# 检查编译结果
if [ $? -eq 0 ]; then
    echo "=========================================="
    echo "增量编译成功完成!"
    if [ $YAML_CHANGED -eq 1 ]; then
        echo "✓ YAML 配置文件已更新到 install 目录"
    fi
    echo "=========================================="
    exit 0
else
    echo "=========================================="
    echo "编译失败，请检查错误信息"
    echo "=========================================="
    exit 1
fi
