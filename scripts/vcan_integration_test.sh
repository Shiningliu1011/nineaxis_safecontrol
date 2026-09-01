#!/usr/bin/env bash
# T07 vcan 9 节点集成验收脚本。
# 需要 root 权限创建 vcan0 接口；无 root 时给出提示。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

# --- 环境自检 ---
check_vcan() {
    if ! ip link show vcan0 &>/dev/null; then
        echo "vcan0 不存在，尝试创建..."
        if ! sudo ip link add dev vcan0 type vcan 2>/dev/null; then
            echo "错误：需要 root 权限创建 vcan0。"
            echo "请运行：sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up"
            echo "或安装 vcan 模块：sudo modprobe vcan"
            exit 1
        fi
        sudo ip link set vcan0 up
        echo "vcan0 已创建并启动。"
    else
        echo "vcan0 已存在。"
    fi
}

check_python_can() {
    if ! python3 -c "import can" 2>/dev/null; then
        echo "警告：python-can 未安装。vcan 集成测试需要它。"
        echo "安装：pip install python-can"
        echo "跳过 vcan 集成测试，仅运行 FakeCANBackend 单测。"
        python3 -m pytest tests/test_socketcan_backend.py -q
        exit 0
    fi
}

check_vcan
check_python_can

# --- 运行集成测试 ---
echo ""
echo "==> 运行 vcan 9 节点集成测试..."
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 -m pytest tests/test_socketcan_backend.py -q -v

echo ""
echo "==> vcan 集成验收完成。"
echo "如需完整验收（延迟<5ms、丢帧<0.1%），请在真机环境运行并检查日志。"
