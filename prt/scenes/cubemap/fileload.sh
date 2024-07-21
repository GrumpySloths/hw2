#!/bin/bash

# 当前目录
current_dir=$(pwd)

# 要复制的路径
target_path="../../../homework2/assets/cubemap/"

# 读取每一行作为目录名
while IFS= read -r dir; do
    # 构建源文件路径
    source_file="$current_dir/$dir/indirect.txt"
    
    # 构建目标文件路径
    destination_file="$target_path$dir/indirect.txt"
    
    # 创建目标目录（如果不存在）
    mkdir -p "$(dirname "$destination_file")"
    
    # 复制文件
    if [ -f "$source_file" ]; then
        cp "$source_file" "$destination_file"
        echo "Copied $source_file to $destination_file"
    else
        echo "File $source_file does not exist."
    fi
done < <(cat <<EOF
CornellBox
GraceCathedral
Indoor
Skybox
EOF
)
