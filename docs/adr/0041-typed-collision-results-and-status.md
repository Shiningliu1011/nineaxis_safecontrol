---
status: accepted
date: 2026-09-21
---

# 碰撞查询返回固定结构结果与明确状态码

`CollisionSafety.query()` 与 `CollisionSafety.certify()` 返回固定 shape 的 typed result。公共
header 包含：

- `status`、`scene_epoch` 和 `scene_revision`；
- `geometry_hash`、`kernel_version` 和碰撞策略身份；
- 查询开始时间、完成时间、运行时间和 deadline 状态。

OSCBF 查询 payload 包含固定容量的 `valid_mask`、`barrier`、`grad_h_q`、
`partial_h_partial_t`、`proximity_scale`、primitive pair identity，以及每行 solver residual、
iteration 和 health。毫米距离 payload 包含当前有符号距离、要求间距、剩余余量、最近 pair、
最近点和 track 状态。区间证明 payload 包含各 segment 的证明状态、下界、二分深度和失败区间。

运行期状态至少区分：

- `OK`；
- `REVISION_PENDING`；
- `INVALID_SCENE`；
- `UNKNOWN_REQUIRED_SPACE`；
- `CAPACITY_OVERFLOW`；
- `CONSTRAINT_OVERFLOW`；
- `SOLVER_UNHEALTHY`；
- `CERTIFICATE_FAILED`；
- `DEADLINE_MISSED`。

只有 `status == OK` 且对应 `valid_mask` 有效的数值可以进入命令 QP、规划边接受或轨迹准入。
其他状态下的数值数组只允许用于诊断，不能通过特殊数值获得安全含义。错误 shape、非法配置、
未知 `query_mode` 和程序调用错误在进入 JAX kernel 前立即抛出异常。

## Decision Basis

固定结构保持 JAX 编译 shape 稳定，并让所有调用方使用相同的失败语义。明确状态可以区分场景
准备、容量、solver、区间证明和 deadline 问题，避免 `NaN`、`inf`、负距离或全零梯度被误当成
有效结果。

## Consequences

命令网关与规划准入必须先检查公共 header，再读取 payload。日志和记录文件保存完整 header、
活动行身份与失败区间。新增状态需要更新 Interface 版本、调用方穷举处理和相关测试。
