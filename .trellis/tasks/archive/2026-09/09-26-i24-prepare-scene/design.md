# 设计

基准：dc0f0cc，工作区原有三个未跟踪目录保持原状。本任务涉及共享碰撞内核与公开输入结构，完成后执行规范与需求审查。

## 数据边界

- 保留 CollisionSafety 三个公开操作。CollisionScene 增加不可省略的元数据、源身份、support 有效 mask、track 关联及固定容量 tracking 数据。
- CollisionIdentities 增加 fixed_environment_revision 与 required_space_hash。prepare_scene 的 identities 参数表示读者期望；scene 保存发布者身份，两者必须一致。
- 元数据明确 base_link、m/mm/m/s/ns、单调时钟、布局版本、需求数量、覆盖空间 identity 与有效期、SHA-256 checksum。
- CollisionScene 提供数据 checksum 方法，供生产者与读者共同使用；采用标准 JSON 与 NumPy NPY 编码，包含字段名、dtype、shape、数据、元数据与源身份，checksum 自身除外。读取方对独立 host snapshot 核验后才复制到 JAX。
- prepare_scene 是 host 准备边界，执行 checksum 与时钟检查；PreparedScene 为仅含固定 shape JAX arrays 的不可变 PyTree。query/certify 消费 PreparedScene，准备操作不再作为 JIT 函数。

## 准入规则

- 采集、发布准备和当前时间均使用同一机器的 time.monotonic_ns。source 到 prepared 的上界为 artifact 的 transport_delay_ns + scene_preparation_delay_ns；source 到本地读取的上界再加 control_delay_ns。时间顺序及 deadline 独立检查。
- required-space hash 与读者期望相同，覆盖布尔值和有效期必须满足当前时间；缺少覆盖返回 UNKNOWN_REQUIRED_SPACE。
- support_count/track_count 表示发布者需容纳的总数量。超过固定容量返回 CAPACITY_OVERFLOW；未超限时必须与 active mask 数量一致。
- TRACKED support 必须关联唯一、活动且有效的 track，速度与 track 一致；track 更新时间、到期时间、误差和加速度范围有效。失效的 TRACKED 输入拒绝，发布者可按规格另行产生 UNTRACKED 场景并保留 occupied。UNTRACKED support 的名义速度为零。
- PreparedScene 保存 valid_until_ns、覆盖期限、全部活动 track 的最早期限及 tracking 数据。查询和证明再次检查时间及 identity；query 用最终 completed_ns 检查三项期限，过期结果不能继续准入。
- tracking 规则共用固定 shape 数组计算；host 准备使用 NumPy，查询复查使用 JAX，不逐项复制 track/support 到 host。
- 失败 status 传递，INVALID_SCENE、CAPACITY_OVERFLOW、UNKNOWN_REQUIRED_SPACE、DEADLINE_MISSED 按明确检查顺序生成。有效 mask 同时受活动状态与成功 status 限制。

## 文件范围

- collision_safety.py：公开类型与操作接入。
- _collision_scene.py：host snapshot、checksum、固定结构验证和场景准备逻辑。
- portable_oscbf/tests：新增场景行为测试，并更新现有真实解析输入以携带完整场景信息。
- docs/modules/portable-oscbf/collision-safety.md：接口与时间语义。

几何资产在构造时已经复制为固定数组，QP 结果已经固定容量；本票核对这些边界并增加跨活动数量的行为证据。
