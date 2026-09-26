# I22 验证记录

日期：2026-09-26。代码基准为 `105be06`，实现提交为 `4788ec81a662d6b6468029790243ebaa51266393`。提交前核对七个代码与测试文件的 SHA-256，内容与最终验收记录一致。执行平台为当前 Linux CPU，JAX/jaxlib 使用项目已安装版本；临时目录全部位于 `.scratch/i22/`。

## 公开行为验收

```bash
TMPDIR=$PWD/.scratch/i22/tmp PYTHONPATH=$PWD/.scratch/i21/python-deps python3 -m pytest portable_oscbf/tests/test_ellipsoid_point.py portable_oscbf/tests/test_ellipsoid_dcol.py portable_oscbf/tests/test_collision_safety.py portable_oscbf/tests/test_collision_parameters.py portable_oscbf/tests/test_collision_geometry_artifact.py -q --basetemp=.scratch/i22/pytest-regression
```

退出码 0；194 passed，270.38 s。日志 `.scratch/i22/regression.log`。其中环境查询 29 项、self DCOL 33 项、接口/参数/几何 132 项。全部调用真实 Module 和几何计算，未使用 mock。

环境查询检查解析球体、接触附近、相交、ellipsoid 表面与内部、中心、重复最短半轴、极端尺寸、旋转和九轴梯度。距离检查独立求解 80 位四维 KKT，并检查 global minimum 对应的 multiplier 区间。随机输入固定 seed 109。空场景、inactive mask、多个 support 与 slot、track 状态、identity、容量、solver limit 和 deadline 均有公开行为断言。

毫米距离字段的 `required_clearance_mm` 与 `rho_mm` 分别验证；距离模式的 `valid_mask`、`state_valid_mask`、`state_valid` 与 barrier/solver 命令字段持续无效。

最终公共 header 的采集时间验证：

```bash
TMPDIR=$PWD/.scratch/i22/tmp PYTHONPATH=$PWD/.scratch/i21/python-deps python3 -m pytest portable_oscbf/tests/test_ellipsoid_point.py portable_oscbf/tests/test_collision_safety.py -q --basetemp=.scratch/i22/pytest-final
```

退出码 0；51 passed，92.58 s。日志 `.scratch/i22/final-query-tests.log`。覆盖最终实现的全部环境查询、query/certify header，以及成功、无效 scene 和 deadline 下的 `source_stamp_ns`。

高精度参考的归一化与内部点初始化检查运行 `test_ellipsoid_point.py -k 'rotated or extreme or seeded or clearance'`，17 passed、12 deselected，63.69 s；日志 `.scratch/i22/reference-tests.log`。

## 质量检查

```bash
TMPDIR=$PWD/.scratch/i22/tmp PYTHONPATH=$PWD/.scratch/i21/python-deps bash scripts/agent_check.sh
python3 -m compileall -q portable_oscbf/work portable_oscbf/tests/test_ellipsoid_point.py
git diff --check
```

退出码均为 0。快速检查中的 69 项纯逻辑测试通过，日志 `.scratch/i22/agent-check.log`。检查包含 Python 编译、纯模块无 ROS import、shell 语法和 diff 空白。

## 计算依据

已完整阅读 David Eberly 的 [Distance from a Point to an Ellipse, an Ellipsoid, or a Hyperellipsoid](https://geometrictools.com/Documentation/DistancePointEllipseEllipsoid.pdf)。内部实现采用局部坐标、单调 multiplier 方程与最短半轴特殊情况，使用平移变量保持内部点的数值精度；测试参考采用独立 KKT Newton 解法。源文件副本和完整提取文本保存在 `.scratch/i22/`。

数值验收标准见 `../implement.md`。本地数值证据不授予生产命令权限。目标设备 deadline、动态误差与活动集合证明、生产接线和最终切换继续由执行地图中的对应票负责。

## 数值 manifest

`TMPDIR=$PWD/.scratch/i22/tmp PYTHONPATH=$PWD/.scratch/i21/python-deps python3 .scratch/i22/numerical_evidence.py` 退出码 0，生成 `numerical-evidence.json`。记录 Python 3.10.12、JAX/jaxlib 0.6.2、mpmath 1.2.1、CPU x86_64、输入状态与半径、数据 SHA-256、geometry/kernel/policy/parameter identity 和最终源码 SHA-256。

三个独立参考案例的最大距离误差为 `2.557954e-13 mm`，robot witness 误差 `7.907670e-9 mm`，support witness 误差 `1.580100e-9 mm`，九轴梯度绝对误差 `1.273951e-8`。距离模式的命令 mask 均为 False。运行时间仅记录这三个本机解析样本，不作为目标设备性能分布或生产准入证明。

## 公开证据索引

[I22 票尾验收记录](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/109#issuecomment-5843597786) 已发布；本票已关闭，[统一执行地图](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/133) 已引用此记录。实现已提交并推送至 `main`。
