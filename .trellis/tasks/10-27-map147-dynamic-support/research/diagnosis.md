# 时间范围与来源下限验证

验证边界为真实 TrackingFrame JSON、LidarRayBatch、occupancy、prepare_scene、query 和直接
tube 查询。独立探针提供确定的解析运动和重新计算 checksum 的输入；两项目标检查可在数秒内
自动执行。原始独立审查材料保持不变。

物理要求：实际更新时间位于记录时刻的误差区间内，更新以前的完整运动需由未跟踪范围包围；
更新以后的名义运动、估计误差及加速度分别计算。每个动态未跟踪来源的半径、增长率、加速度
与有效期限须至少覆盖实际 CollisionPolicy 的原观测年龄范围。group 对成员的包围和 policy
对来源的检查分别执行。

源码对应关系由独立探针直接给出，因此采用解析边界测试验证，不增加无关诊断分支。修复前
20 个文件与独立审查清单逐一核验，记录在工作区 `.scratch/map147-101-review/before-handoff.json`
及 `before-source-sha256.txt`。原独立两项检查的 red 命令与 JUnit 存于同目录 `red-original.*`。

policy 参数由模块自身不可变配置生成固定数组，输入场景无法提供或替换；直接 tube 查询需要
显式 CollisionPolicy，缺少时保持无效。静态 scene、occupancy 清除及 certify 拒绝的行为保留。

## 固定模型来源与输入拒绝

固定来源需要实际 FixedEnvironmentModel 的完整内容身份与原始几何。模型绑定 policy、
revision、来源与验证说明；每个固定成员的 hash 和几何由该模型核验。零采集时间与输入类型
声明均不能单独提供固定来源。模型复制、序列化与 host 查询均执行内容身份验证，并作为 JAX
static PyTree 参与编译。register_static 的用法依据本机 JAX 0.6.2 的
`jax/_src/tree_util.py:1091` 完整实现。

snapshot 对候选有效原观测完成 tracking 类型、身份、关联和内容检查后，才执行过期处理和
内部时间、revision 更新。验证包括跨越观测期限的拒绝请求，以及随后使用原有效时间查询。

修订前 21 个文件与 review-fix-handoff.json 全部相同，保存在工作区
`.scratch/map147-101-review2/before-handoff.json`。未修改的独立 Spec 与 Standards 探针中
三项目标检查为 3 failed；新增公开接口测试为 10 failed。修订后分别为独立 11 passed 和
对应 10 passed。完整命令、原始结果及最终验证另存同一目录。

## 全部固定成员的完整覆盖

模型至 source 的核验须同时满足每个模型成员都出现于有效 source；模型至 group 的核验须
覆盖全部原始成员球。只有逐 source 的来源匹配无法说明模型的完整表示。旧静态布局没有
时间来源数组，仍以有效 group 的几何覆盖核验全部实际模型成员。

未修改独立探针在修订前为 1 failed；新增测试为 4 failed、2 passed，覆盖关闭全部来源、
遗漏一个成员、关闭整个 group、旧静态 group 遗漏及合法三成员合并。修订后新增 6 项与完整
独立 20 项通过。合法合并的预期采用原始模型成员球的独立几何包含公式，并检查 101 个时刻。
原始结果与来源身份保存在 `.scratch/map147-101-review3`。
