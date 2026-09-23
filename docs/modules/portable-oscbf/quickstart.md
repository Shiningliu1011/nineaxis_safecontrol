## 快速开始

以下相对路径命令在仓库的 `portable_oscbf/` 目录执行。

### 安装依赖

```bash
python3 -m pip install -r portable_oscbf/requirements.txt
python3 -m pip install numpy scipy python-fcl trimesh osqp pyyaml urdf-parser-py
```

### 基本使用

```python
import sys
sys.path.insert(0, '.')  # 使 'work' 包可导入

import numpy as np
from work.jax_control_facade import JaxControlLoop

# 1. 创建控制循环
ctrl = JaxControlLoop(
    dt=0.002,              # 控制周期 2ms
    w_pos=20.0,            # 位置权重
    w_orient=10.0,         # 姿态权重
    w_joint=0.1,           # 零空间关节权重
    enable_x64=True,       # JAX float64
)

# 2. 准备路径参考 (从 .mat 加载)
from work.ik_data_loader import load_trajectory
traj = load_trajectory('data/ik_input.mat')

# 3. 设置路径跟踪
ctrl.setup_path_tracking(
    path_geometry=traj.path_geometry,
    path_config=traj.path_config,
)

# 4. 运行控制循环
q = np.zeros(9)  # 初始关节角
for step in range(num_steps):
    # 准备障碍物数据 (固定 shape)
    obs_pos = ...      # (MAX_OBS, 3)
    obs_radii = ...    # (MAX_OBS,)
    obs_enabled = ...  # (MAX_OBS,) 1=启用, 0=禁用

    # 执行一步
    result = ctrl.path_tracking_step(
        q=q,
        obs_pos=obs_pos,
        obs_radii=obs_radii,
        obs_enabled=obs_enabled,
    )

    q = result.q_next  # 下一步关节角
    # result.u_safe    → 安全关节速度
    # result.err_6d    → 6D 任务误差
    # result.qp_ok     → QP 是否成功
    # result           → 一步的完整记录 (内核侧字段见 work/control_step_record.py)
    # result.min_obs_dist_measured → False 时 min_obs_dist 是占位值, 不是观测
```

### 运行测试

```bash
cd portable_oscbf
python -m pytest tests/ -v
```
