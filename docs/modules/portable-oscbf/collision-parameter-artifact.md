# CollisionParameterArtifact

`portable_oscbf/work/collision_parameters.py` 提供版本化参数加载、`parameter_hash` 和
`collision_policy_hash`。格式定义在
[`collision_parameter_schema.json`](../../../portable_oscbf/config/collision_parameter_schema.json)，
依据 [ADR 0045](../../adr/0045-versioned-collision-parameter-artifact.md)。依赖为
`jsonschema==4.23.0`，随内核运行依赖安装。

## 文件内容

JSON 顶层固定为 `schema_version=1`、`generator`、`input_data_hashes`、`parameters`、
`self_clearance_mm`、`parameter_hash`。未知字段与不支持的版本均拒绝加载。

`parameters` 保存以下数值，字段全名、类型和单位以 schema 为准：

| 类别 | 内容与单位 |
| --- | --- |
| 误差 | LiDAR 距离、入射方向、外参平移、移动关节插值使用 `mm`；LiDAR 角度、外参旋转、旋转关节插值使用 `rad` |
| 时间 | 逐点时间误差、传输、场景准备和控制延迟使用整数 `ns` |
| support | voxel 尺寸、support 半径上限、合并半径上限使用 `mm`；合并数量使用整数 |
| 运动范围 | 未跟踪障碍物速度和加速度上界分别使用 `mm/s` 与 `mm/s^2` |
| 执行误差 | 命令位置误差、反馈误差和停车距离使用机器人空间包络的 `mm` 上界 |
| 间距与响应 | environment 间距使用 `mm`；self/environment CBF 响应率使用 `s^-1` |
| 容量 | support、track、每个 ellipsoid 的 environment pair、每个 link 的 ellipsoid、primitive row、query batch、segment batch 使用正整数，单位 `1` |
| solver | DCOL、QP、距离求解 iteration 使用正整数；residual 使用无量纲正数；距离求解容差使用 `mm` |
| 区间证明 | 二分深度使用非负整数 |
| deadline | 场景准备、device transfer、query、QP、完整控制周期、certify、剩余轨迹验证与重新规划均使用正整数 `ns` |

`self_clearance_mm` 是 link pair 数组，每项保存两个不同 link 的 `links` 和独立 `clearance`
记录。重复 pair（含逆序）会被拒绝。几何消费者接入时还须核对全部候选 pair 都具备间距记录。

每条参数记录都必须提供：

- `value` 与固定 `unit`；
- `source.description` 与 `source.data_hashes`；
- `measurement_method`、`calculation_method`；
- `devices`、`scenarios` 适用范围；
- `confidence_requirement`；
- `validation.data_hashes`、`validation.method`、`validation.result="passed"` 和 `validation.evidence`。

顶层 `input_data_hashes` 必须精确包含全部来源与验收数据 hash。所有开发数据与验收数据的
hash 集合必须互不相交。`generator` 保存工具名称、版本和生成方法。来源及验收记录使用
SHA-256 的 64 字符小写十六进制形式。

加载器拒绝非有限值、负范围、错误单位、空元数据、非整数容量与时间、超出整数表示范围、
设备或场景不适用及声明 hash 不匹配。query 与 QP deadline 之和不能超过控制 deadline，
完整控制 deadline 不能超过 100 Hz 的 10 ms；support 半径上限必须能够覆盖 voxel 半对角线
及声明的合并半径上限。

## 生成与使用

离线生成工具把测量及独立验收结果整理为不含 `parameter_hash` 的 payload，调用
`CollisionParameterArtifact.create(payload, device=..., scenario=...)`，通过 `to_document()`
取得完整 JSON 内容并写出文件。运行加载始终要求文件自带正确的 `parameter_hash`。

在 `portable_oscbf` 可以导入的环境中：

```python
from work.collision_parameters import CollisionParameterArtifact, CollisionPolicy
from work.collision_safety import CollisionSafety, CollisionSafetyConfig

artifact = CollisionParameterArtifact.load(
    parameter_path, device=device_identity, scenario=scenario_identity
)
policy = CollisionPolicy(
    artifact=artifact,
    geometry_hash=bytes.fromhex(verified_geometry["geometry_hash"]),
    kernel_version=kernel_identity,
    allowed_contacts=tuple(verified_geometry["self_collision"]["allowed_contacts"]),
)
module = CollisionSafety(CollisionSafetyConfig.from_policy(policy))
```

geometry 与 kernel identity 均为 32 字节。允许接触记录沿用 geometry artifact 的
`links`、`reason`、`evidence`、`geometry_hash`；加载时检查结构、pair 与 geometry 绑定。
允许接触的几何证明由几何资产生成与验收流程负责。当前生产几何资产的允许接触列表为空。

`parameter_hash` 覆盖除其自身外的完整 artifact，`collision_policy_hash` 覆盖该 hash、
geometry identity、kernel identity、完整允许接触列表及 policy 格式版本。计算使用
排序 object key、紧凑 UTF-8 JSON 和 SHA-256。空白与 object key 顺序不影响 identity；
数组顺序与数值类型保留在 identity 中。

artifact、policy、配置及内部容器均不可变；`to_document()` 返回独立副本。配置的全部容量和
deadline 必须与 artifact 一致，`CollisionSafety.config` 只读。运行期间改变参数需要生成
新的 artifact 和 policy，并重新建立模块与场景准入。

三个公开操作都会核对当前 geometry/kernel/policy。`prepare_scene` 还检查有效 support 的
clearance 等于 artifact 的 environment clearance、半径不超过 artifact 上限。
旧 identity 的场景及 `PreparedScene` 被拒绝；query 与 certify 保持无效 mask，状态为
`INVALID_SCENE`，超过 deadline 时状态为 `DEADLINE_MISSED`。

## 当前验证边界

本实现验证数据格式、来源记录、适用范围、身份和公开操作的拒绝行为。仓库尚无经过目标设备
测量及独立验收的生产参数文件。测试输入只适用于 `analytic-interface-test` 设备和
`json-and-identity-validation` 场景，其结果证明软件接口行为。

实际误差上界、设备性能、数据内容真实性与置信要求是否满足，需要参数测量和独立验收证据。
后续规划、恢复和验收 manifest 消费者须在使用既有结果时比较完整 identity。当前 query 与
certify 的数值计算及运动准入状态见 [CollisionSafety 接口](collision-safety.md)。

验证入口为 `portable_oscbf/tests/test_collision_parameters.py`、
`portable_oscbf/tests/test_collision_safety.py` 与依赖安装一致性测试。
