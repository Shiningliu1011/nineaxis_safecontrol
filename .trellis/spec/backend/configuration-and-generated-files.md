# 配置与生成文件

## 生产配置

`config/oscbf_controller.yaml` 是 OSCBF ROS 入口的生产配置。`src/robot_safecontrol_moveit/production_config.py` 在创建控制资源和 command publisher 前读取并验证该文件。

处理顺序必须保持：

1. 读取指定 node 的 `ros__parameters`。
2. 检查必需字段、未知字段、旧字段、数据类型、有限值和取值范围。
3. 确认 YAML 基础配置本身有效。
4. 合并明确提供的 ROS override，并再次验证最终值。
5. 解析资源路径，确认文件或目录存在且可读。
6. 把值、来源、override 链和资源解析结果写入不可变的 `EffectiveConfiguration`。
7. 创建 controller、subscription 和 publisher。

核心类型采用冻结 dataclass 和只读 mapping：

```python
@dataclass(frozen=True)
class EffectiveConfiguration:
    values: Mapping[str, Any]
    sources: Mapping[str, str]
    override_chains: Mapping[str, list[dict[str, Any]]]
    resources: Mapping[str, Mapping[str, str]]
```

必需 YAML 字段不能依靠 override 补齐。未知字段和已经废弃的控制字段必须报告错误。新增字段时，需要同步修改：参数集合、类型与范围校验、实际消费者、生产 YAML、运行快照和配置测试。

## Launch 与 node 参数

- launch 负责进程拓扑、参数文件路径、显式 mode 和 remap。
- node 负责验证自身参数，并在创建 I/O 前拒绝不安全组合。
- 最终 launch 默认 `hardware_mode=sim`，仅接受 `sim` 与 `shadow`；`live` 在任何进程启动前被拒绝。
- 参数来源必须可追踪到 YAML、可选默认值或显式 override。
- 默认 launch 与 `run_demo.sh` 的启动行为不同，不能把演示脚本行为写成生产默认值。

相关证据：`launch/mujoco_transition_final.launch.py`、`tests/test_launch_structure.py`、`tests/test_oscbf_production_config.py`。

## 生成文件

受管文件应由对应脚本重新生成，禁止直接编辑生成结果。

- `models/ninezzhou/` 中的 URDF 是运动学来源。运行 `python3 portable_oscbf/scripts/generate_kinematics_data.py` 更新 `portable_oscbf/work/kinematics_data.py`，并运行同一脚本的 `--check` 模式验证一致性。
- mesh 与采样规则通过 `portable_oscbf/scripts/generate_obb_calibration.py` 生成 `portable_oscbf/work/obb_collision_model.py` 和 `portable_oscbf/config/obb_model.yaml`，使用 `--check` 验证。
- `portable_oscbf/config/dcol_alpha.yaml` 由 `portable_oscbf/scripts/calibrate_dcol_alpha.py` 生成，其来源数据和适用范围要保留在文件元数据中。

代表性测试：

- `portable_oscbf/tests/test_generated_kinematics_data.py`
- `portable_oscbf/tests/test_fk_matches_urdf.py`
- `portable_oscbf/tests/test_obb_model.py`
- `tests/test_mujoco_obb_visualization.py`

## 构建与安装副本

修改 `aeb_moveit_plugin/` 中的 C++ 插件后，运行 `bash build_aeb_moveit.sh`。随后重新加载 `install/setup.bash` 并重启本任务的 launch，因为运行中的 `move_group` 不会重新加载共享库。

Python、launch、config、data 和 `portable_oscbf` 的安装内容由 `setup.py` 管理。新增运行资源时，要同时确认源码路径和安装后的 package share 路径都可用，并补充安装场景测试。

## 禁止模式

- 在控制器已经创建资源后再验证生产配置。
- 允许未知字段静默进入 parameter 集合。
- 使用 override 修补缺少必需字段的 YAML。
- 复制生成常量到另一个模块。
- 直接修改 `kinematics_data.py` 或受管 OBB 文件。
- 只验证源码目录，忽略安装后的 package share。
