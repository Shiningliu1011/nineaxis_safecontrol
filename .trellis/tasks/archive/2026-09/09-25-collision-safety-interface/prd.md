# 固定 CollisionSafety 公开接口与 typed result

## Goal

建立供规划、轨迹执行和 OSCBF 共用的无 ROS JAX `CollisionSafety` 接口，范围以 [固定 CollisionSafety 公开接口与 typed result](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/103) 和 [CollisionSafety 统一碰撞安全模块规格说明书](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/98) 的决定 1、2、33 至 37 为准。

## Requirements

- `CollisionSafety` 仅公开 `prepare_scene(scene, identities)`、`query(query_batch, prepared_scene, query_mode)` 和 `certify(segment_batch, prepared_scene)` 三个操作。接口不得导入 ROS。
- 场景、状态批次与区间批次使用固定 shape；结果使用 JAX 可识别的固定字段 typed result。非法 shape、非法配置、未知 `query_mode` 和调用方式错误立即抛出异常。
- `prepare_scene` 接受 `float32` LiDAR 坐标，并在 Module Interface 内转成 `float64`；参与碰撞计算的数值保持 `float64`。非有限场景数值产生明确的无效状态，非有限机器人状态或轨迹输入立即拒绝。
- `query` 和 `certify` 的公共 header 携带 status、scene epoch/revision、geometry、kernel、policy identity、开始时间、完成时间、运行时间与 deadline 状态。
- `query` 的 OSCBF payload 保留固定容量的 `valid_mask`、barrier、`grad_h_q`、`partial_h_partial_t`、`proximity_scale`、primitive pair identity、solver residual、iteration 和 health；毫米距离及状态有效性使用固定结果字段。
- `certify` 返回固定容量的区间结果，包括有效 mask、区间证明状态、下界、二分深度和失败区间。
- 公开状态至少覆盖 `OK`、`REVISION_PENDING`、`INVALID_SCENE`、`UNKNOWN_REQUIRED_SPACE`、`CAPACITY_OVERFLOW`、`CONSTRAINT_OVERFLOW`、`SOLVER_UNHEALTHY`、`CERTIFICATE_FAILED` 和 `DEADLINE_MISSED`。非 `OK` 的数值结果只能用于诊断，不能获得命令或轨迹准入。
- 本票保留现有生产碰撞路径的命令权限边界；几何资产、具体碰撞 kernel、场景服务和连续区间证明由统一执行地图中的相应后续票实现。
- 当前缺少真实碰撞计算与连续区间证明时，合法的 `query`、`certify` 调用返回明确的非 `OK` 状态，全部准入 mask 保持无效。此项已由用户在本会话确认。

## Acceptance Criteria

- [x] 在不加载 ROS 的进程中导入并调用三个公开操作；JAX 编译路径与结果 PyTree 保持固定结构。
- [x] 接口行为测试覆盖合法输入、固定 mask、非法 shape、非有限值、未知模式和所有公开 status。
- [x] 对应结果字段具有固定 shape、dtype 和明确的有效性字段；任何未完成碰撞计算或证明的结果都不能表示准入成功。
- [x] 文档说明三个操作、输入数据、结果结构、精度和当前命令权限边界。

## Notes

- 本票属于 [速度级 OSCBF 与 CollisionSafety 统一执行地图（仅激光雷达感知）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/133)。
