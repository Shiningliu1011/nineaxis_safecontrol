# 规范与需求审查

基准为 `main` 的 `2440192f04b3b9d59b5d10666a589c1a2970eb1c`，检查本票工作区差异及新增文件。
遵循本会话 Codex inline 要求，由主会话分别完成规范和需求检查。

## 规范

- 参数模块仅使用标准库与明确声明的 jsonschema，CollisionSafety 继续保持无 ROS 边界。
- JSON 由标准库解析，Draft 2020-12 由 jsonschema 检查；重复字段、未知字段、不完整记录、非有限数值和非法单位直接拒绝。
- artifact、policy、config 使用冻结 dataclass，嵌套 mapping 与 sequence 使用只读对象；对外导出的 JSON 文档为独立副本。
- 全部字段通过 `parameter_hash` 绑定；policy 对 geometry/kernel identity、接触记录及参数 identity 再次计算 SHA-256。
- 查询和证明只执行共同的 scene 数值与 identity 谓词，参数文件与 schema 读取只发生在模块建立阶段。
- 当前生产控制入口和硬件模式没有改动。新增依赖已经同步到 manifest、setup.py 和版本检查；wheel 中包含 schema。
- 文档说明格式、调用方式、适用范围和当前验证边界。

## 需求

- schema 覆盖 ADR 0045 所列参数类别，每条记录包含单位、来源、方法、设备、场景、置信要求及独立验证。
- 测试逐个修改 46 个安全参数，验证 parameter/policy identity 变化和旧 identity 拒绝；另外覆盖 pair clearance、生成方法、数据身份和 provenance。
- 独立 Python 进程及 wheel 安装目录产生相同 identity。
- `CollisionSafetyConfig.from_policy` 从 artifact 取得现有接口所需值，直接构造也必须绑定一致的 policy。
- `prepare_scene`、`query`、`certify` 共同拒绝旧 policy、geometry、kernel 和允许接触列表产生的旧 PreparedScene。
- 现有接口的 status、固定 mask、shape/dtype、JAX 准备路径和无 ROS 行为继续由原接口测试覆盖。

## 剩余范围

实际设备测量、独立验收数据及完整候选 pair 覆盖核对由参数测量和几何消费者接入任务负责。将来规划、恢复及验收证据的消费者需要比较完整 identity。当前接口没有运动授权能力，本票验收仅覆盖参数与身份基础设施。

本票检查未发现遗留的规范或需求问题。实际验证命令和输出见 `implement.md`。
