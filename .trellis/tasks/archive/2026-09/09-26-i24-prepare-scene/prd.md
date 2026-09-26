# 场景准入

来源：[实现 prepare_scene 与 PreparedScene 准入](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/100)。规格采用已确认的 CollisionSafety 决定 3、14、15、33、34、36、37，以及 ADR 0034、0036、0037、0039、0041。

## 要求

- 验证 frame、单位、布局、有限值、epoch/revision、checksum、geometry/kernel/policy identity、采集与准备时间、required-space coverage、容量、固定模型 revision 和 tracking 有效期。
- 成功结果不可变，浮点输入统一转换为 float64；support、track、robot geometry 和查询结果的容量在模块生命周期内固定，活动数据由 mask 表达。
- 非 OK 状态保持明确含义，query 与 certify 不能从失败场景得到有效准入 mask。
- shape、dtype 和调用错误在计算前抛出异常。容量超限不能通过裁剪得到 OK。

## 验收

- 有效输入得到 OK；逐项错误输入得到异常或对应公开非 OK 状态。
- checksum 覆盖元数据、源身份和全部固定容量数组；读者期望身份与源身份逐项比较。
- 数据年龄、未来时间、覆盖失效、过期 tracking 和固定模型 revision 不匹配不能得到 OK。
- 改变原始 NumPy 输入不能改变 PreparedScene；不同活动数量的结果 PyTree shape 相同，支持 JAX 消费。
- 现有 DCOL、point-scale、毫米距离、参数 identity 回归通过，文档说明输入、时间与失败行为。

## 边界

本票实现场景读取端准入。场景服务、共享内存原子发布、support 合并、动态 tube、活动集合证明及生产命令切换由各自执行票负责。
