## 移植到新项目

### 步骤 1: 复制控制内核与模型来源

```bash
mkdir -p /path/to/new_project/models
cp -r portable_oscbf /path/to/new_project/
cp -r models/ninezzhou /path/to/new_project/models/
```

保持 `portable_oscbf/` 与 `models/ninezzhou/` 的相对目录结构。后者包含生成运动学常量所需的 URDF 与 STL 网格。

### 步骤 2: 替换 URDF 并生成运动学常量

替换 `models/ninezzhou/urdf/ninezzhou.urdf` 与对应网格，然后从项目根目录运行：

```bash
python3 portable_oscbf/scripts/generate_kinematics_data.py
python3 portable_oscbf/scripts/generate_kinematics_data.py --check
```

生成结果写入 `portable_oscbf/work/kinematics_data.py`，包含 `JOINT_CHAIN`、关节位置限幅与关节数量。两个运动学模块直接读取该文件。轨迹配置加载时校验 `nineaxis.yaml` 的关节顺序；速度及加速度限幅由 `actuator_modules.yaml` 提供。

### 步骤 3: 修改碰撞模型

替换 STL 网格或调整 `scripts/generate_obb_calibration.py` 中各连杆的
`SAMPLE_GRID_SHAPES`，然后从项目根目录运行：

```bash
python3 portable_oscbf/scripts/generate_obb_calibration.py
python3 portable_oscbf/scripts/generate_obb_calibration.py --check
```

生成结果写入 `work/obb_collision_model.py` 与 `config/obb_model.yaml`。
32 个环境采样球由 OBB 分格直接生成；球心取格心，半径取格半对角线并加
2 mm。

### 步骤 4: 替换轨迹数据

将新机器人的轨迹 `.mat` 文件放入 `data/`, 或修改 `ik_data_loader.py` 适配新的数据格式。

### 步骤 5: 调整配置

编辑 `config/` 下的 YAML 文件:
- `nineaxis.yaml` — 轨迹标定、几何和控制参考配置；`joint_names` 与项目关节顺序一致
- `actuator_modules.yaml` — 执行器限位
- `obb_model.yaml` — OBB 包络数据
- `obstacle_params.yaml` — 障碍物参数

生产 ROS `oscbf_controller` 的参数基础文件位于仓库根目录
`config/oscbf_controller.yaml`。内核回归轨迹 `data/ik_input.mat` 会由根目录
`setup.py` 安装到 package share 的 `portable_oscbf/data/`。

### 步骤 6: 集成到新控制框架

`JaxControlLoop` 是纯计算类, 无 ROS 依赖。只需:

```python
from work.jax_control_facade import JaxControlLoop

# 在你的控制框架中初始化
ctrl = JaxControlLoop(dt=0.002, ...)

# 每个控制周期调用
result = ctrl.path_tracking_step(q, obs_pos, obs_radii, obs_enabled)
q_next = result.q_next
```
