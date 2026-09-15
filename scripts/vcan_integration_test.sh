#!/usr/bin/env bash
# Real Linux vcan only. Never falls back to Fake/virtual and never alters can0.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
if ! python3 -c 'import can' >/dev/null 2>&1; then
    echo 'NOT RUN: install python-can==4.6.1 first.'
    exit 2
fi
if ! ip -details -json link show vcan0 2>/dev/null | python3 -c '
import json,sys
s=json.load(sys.stdin)[0]
sys.exit(0 if s.get("linkinfo",{}).get("info_kind")=="vcan" and "UP" in s.get("flags",[]) else 1)
' 2>/dev/null; then
    echo 'NOT RUN: requires an UP vcan0 (not a physical CAN interface).'
    echo 'One-time setup: sudo modprobe vcan'
    echo '                sudo ip link add dev vcan0 type vcan'
    echo '                sudo ip link set vcan0 up'
    exit 2
fi
HARDWARE_TEST_VCAN=vcan0 python3 -m pytest tests/test_python_can_backend.py -q -k linux_vcan
