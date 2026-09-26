# 设计

## 当前行为与修改位置

`work/collision_safety.py` 接受调用方提供的容量、deadline 和三个 32 字节 identity。尚无参数 artifact；`prepare_scene` 检查 identity，`query` 与 `certify` 对已准备 scene 仅检查结构。本票在无 ROS 内核增加参数加载边界，并使三个操作统一检查当前 identity。

## 数据与接口

- 新增 `work/collision_parameters.py`：使用标准库 JSON 与 jsonschema Draft 2020-12；schema 随 `config/collision_parameter_schema.json` 安装。固定版本拒绝未知字段，各参数具有独立 provenance。有限值及参数间关系在加载边界检查。
- 参数记录包含 `value`、`unit`、`source`、`measurement_method`、`calculation_method`、`devices`、`scenarios`、`confidence_requirement`、`validation`。validation 记录独立数据 hash、方法、结果和证据。开发与验证数据 hash 不相交。
- `CollisionParameterArtifact.load(path, device=..., scenario=...)` 验证适用范围、内容及声明的 hash，返回深层不可变对象。离线 `create` / `to_document` 支持生成工具写出合法文件，hash 由完整内容计算，不包含声明的 `parameter_hash`。
- policy 绑定 artifact、32 字节 geometry/kernel identity 以及带 link identity、原因、验证依据和 geometry hash 的允许接触列表。采用排序 key、紧凑 UTF-8 JSON 与 SHA-256，保留数组顺序；相同文件内容独立于空白及 object key 顺序。
- `CollisionSafetyConfig.from_policy` 从绑定后的参数取得容量和 deadline，持有不可变 policy。所有 config 构造都要求 policy，并拒绝任何值与其不一致的构造。
- `CollisionSafety.config` 为只读属性。绑定配置检查 scene clearance 与 artifact 一致。`query` 和 `certify` 重新检查 geometry/kernel/policy，旧 PreparedScene 返回 `INVALID_SCENE`。

## 文件范围

修改内核参数模块、schema、CollisionSafety、对应行为测试、依赖清单和安装配置、模块使用说明。新增依赖为 `jsonschema==4.23.0`。不修改生产控制配置和硬件模式，也不提供未经测量的生产参数文件。

## 风险与验证

共享内核 identity 属于安全相关范围，完成 Trellis 检查及主会话 code-review。参数测试使用明确标记的解析软件测试输入，验证数据结构与身份，不代表真机参数合格。实际测量证据的科学有效性由后续验收任务负责；加载器验证结构、来源身份与范围。
