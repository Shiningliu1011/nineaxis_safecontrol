# 路径跟踪评价报告

- 任务质量：不通过；在线门控：证据不足
- 全部所需判据：不通过（仅限下述证据范围，不授予实机准入）
- 证据：model；测量边界：kernel_candidate
- 运行：oscbf-runtime-20260912T143705.449275Z-f0bdbc946d90499bae7cc85ea0928090；工况：configured path; obstacles=False
- 测量定义：post-integration model q_next and command reference; QP rows at solve input; before command filter; no execution feedback；时间基准：perf_counter sample start; step_once includes kernel and host diagnostics
- 模型 / 配置 / 路径 / 数据身份：/tmp/pytest-of-lsn/pytest-12/oscbf_m100/runtime_snapshots/oscbf-runtime-20260912T143705.449275Z-f0bdbc946d90499bae7cc85ea0928090.json#software / /tmp/pytest-of-lsn/pytest-12/oscbf_m100/runtime_snapshots/oscbf-runtime-20260912T143705.449275Z-f0bdbc946d90499bae7cc85ea0928090.json / sha256:a4aa60dbde62d798d37266882556a6103ba401d1f3e449b6eb9d7cc1641bfc91 / /tmp/pytest-of-lsn/pytest-12/oscbf_m100/runtime_snapshots/oscbf-runtime-20260912T143705.449275Z-f0bdbc946d90499bae7cc85ea0928090.json#kernel_step_sequence
- 总样本：1；终止原因：completed
- 样本记录 SHA-256：68a96529344fbcf420f48072a07cbace67bdc7de55f0a7ad8a8ec32bfe13ca19
- 声明范围：EvaluationScope(total_length_m=1.8167634936816308, start_m=0.0, end_m=1.8167634936816308)；初始/最终投影弧长：1.17792 / 0.00348992 m
- 绝对弧长比：0.00192095；本次区间覆盖比：0
- 本次区间完成：False；整条路径覆盖：False；全部所需判据下整条路径验证：False
- 采样耗时：0 s；计算 deadline：20 ms；超期率：0
- 未采集边界：filtered_command, simulated_state, real_feedback

## 误差及诊断统计

统计按样本等权；部分有效样本仍可供诊断，缺失/无效样本阻止对应指标通过。

| 指标（单位见定义） | 有效/缺失/无效 | 均值 | RMS | p95 | 最小 | 最大 |
|---|---|---|---|---|---|---|
| actual_tangent_speed_m_s | 1/0/0 | -0.013351 | 0.013351 | -0.013351 | -0.013351 | -0.013351 |
| cross_track_m | 1/0/0 | 0.744135 | 0.744135 | 0.744135 | 0.744135 | 0.744135 |
| delta_slack | 1/0/0 | 6.36478e-11 | 6.36478e-11 | 6.36478e-11 | 6.36478e-11 | 6.36478e-11 |
| feedrate_m_s | 1/0/0 | 0 | 0 | 0 | 0 | 0 |
| legacy_orientation_error | 1/0/0 | 1 | 1 | 1 | 1 | 1 |
| obstacle_clearance_m | 0/1/0 | 未测 | 未测 | 未测 | 未测 | 未测 |
| obstacle_margin_m | 0/1/0 | 未测 | 未测 | 未测 | 未测 | 未测 |
| online_cross_track_m | 1/0/0 | 0.535198 | 0.535198 | 0.535198 | 0.535198 | 0.535198 |
| pos_error_m | 1/0/0 | 0.744135 | 0.744135 | 0.744135 | 0.744135 | 0.744135 |
| qp_primal_residual | 1/0/0 | 0 | 0 | 0 | 0 | 0 |
| source_time_s | 1/0/0 | 0 | 0 | 0 | 0 | 0 |
| step_latency_ms | 1/0/0 | 0.003019 | 0.003019 | 0.003019 | 0.003019 | 0.003019 |
| tool_axis_error_rad | 1/0/0 | 1.57095 | 1.57095 | 1.57095 | 1.57095 | 1.57095 |

## 逐项判据与来源

| 指标 | 类别 | 阈值/单位 | 状态 | 测量值 | 结论 | 来源/原因 |
|---|---|---|---|---|---|---|
| cross_track_m.rms | task | <= 0.00015 m | accepted | 0.744135 | 不通过 | GitHub #14 resolution 2026-09-04; complete samples compared with sourced threshold |
| cross_track_m.p95 | task | <= 0.0005 m | accepted | 0.744135 | 不通过 | GitHub #14 resolution 2026-09-04; complete samples compared with sourced threshold |
| cross_track_m.maximum | task | <= 0.002 m | accepted | 0.744135 | 不通过 | GitHub #14 resolution 2026-09-04; complete samples compared with sourced threshold |
| tool_axis_error_rad.rms | task | <= 0.000872665 rad | accepted | 1.57095 | 不通过 | GitHub #14 resolution 2026-09-04; OFF-15 user decision: true axis angle 2026-09-12; complete samples compared with sourced threshold |
| tool_axis_error_rad.maximum | task | <= 0.00872665 rad | accepted | 1.57095 | 不通过 | GitHub #14 resolution 2026-09-04; OFF-15 user decision: true axis angle 2026-09-12; complete samples compared with sourced threshold |
| qp_success_rate | task | >= 0.999 1 | accepted | 1 | 通过 | GitHub #14 resolution 2026-09-04; complete samples compared with sourced threshold |
| obstacle_clearance_nonnegative | safety | >= 0 m | accepted | 未测 | 证据不足 | GitHub #14 resolution 2026-09-04; nonnegative clearance alone does not prove non-overlap; required samples missing or invalid |
| obstacle_clearance_m.minimum | safety | >= 0.03 m | provisional | 未测 | 证据不足 | GitHub #14 resolution 2026-09-04; geometry/scope pending #8/#18; threshold not accepted for this scope |
| deadline_miss_rate | timing | <= 0.01 1 | provisional | 0 | 证据不足 | GitHub #14 resolution 2026-09-04; deadline must match current measured boundary/config; threshold not accepted for this scope |

## 在线门控与约束

- QP：success=1，失败=0，缺失/无效=0
- 准入：{'unmeasured': 1}；重叠：{'unmeasured': 1}
- 状态事件：{'reference_endpoint_hold': 1}；限制因素：{4: 1}
- 几何裕度、原始约束残差和松弛分别报告；不同量纲不相加。

- legacy_orientation_error 是旧控制误差诊断；delta_slack/qp_primal_residual 是跨类求解诊断，不能当作米解释，物理量请使用逐类数据。

```json
{
  "esdf.residual": {
    "quantity": "max_positive_unrelaxed_G_u_candidate_minus_h",
    "unit": "m/s",
    "source": "QP solve state; dynamic-corrected RHS; before health gate/integration/filter",
    "statistics": {
      "count": 0,
      "missing": 1,
      "invalid": 0,
      "mean": null,
      "rms": null,
      "p95": null,
      "minimum": null,
      "maximum": null
    },
    "inactive_count": 1
  },
  "esdf.static_margin": {
    "quantity": "static_cbf_rhs_div_baseline_alpha",
    "unit": "m",
    "source": "QP solve state; static RHS before dynamic correction; model metric, not physical clearance",
    "statistics": {
      "count": 0,
      "missing": 1,
      "invalid": 0,
      "mean": null,
      "rms": null,
      "p95": null,
      "minimum": null,
      "maximum": null
    },
    "inactive_count": 1
  },
  "joint_angular.residual": {
    "quantity": "max_positive_unrelaxed_G_u_candidate_minus_h",
    "unit": "rad/s",
    "source": "QP solve state; dynamic-corrected RHS; before health gate/integration/filter",
    "statistics": {
      "count": 1,
      "missing": 0,
      "invalid": 0,
      "mean": 0.0,
      "rms": 0.0,
      "p95": 0.0,
      "minimum": 0.0,
      "maximum": 0.0
    },
    "inactive_count": 0
  },
  "joint_angular.static_margin": {
    "quantity": "static_cbf_rhs_div_baseline_alpha",
    "unit": "rad",
    "source": "QP solve state; static RHS before dynamic correction; model metric, not physical clearance",
    "statistics": {
      "count": 1,
      "missing": 0,
      "invalid": 0,
      "mean": 0.4306072000000001,
      "rms": 0.4306072000000001,
      "p95": 0.4306072000000001,
      "minimum": 0.4306072000000001,
      "maximum": 0.4306072000000001
    },
    "inactive_count": 0
  },
  "joint_linear.residual": {
    "quantity": "max_positive_unrelaxed_G_u_candidate_minus_h",
    "unit": "m/s",
    "source": "QP solve state; dynamic-corrected RHS; before health gate/integration/filter",
    "statistics": {
      "count": 1,
      "missing": 0,
      "invalid": 0,
      "mean": 0.0,
      "rms": 0.0,
      "p95": 0.0,
      "minimum": 0.0,
      "maximum": 0.0
    },
    "inactive_count": 0
  },
  "joint_linear.static_margin": {
    "quantity": "static_cbf_rhs_div_baseline_alpha",
    "unit": "m",
    "source": "QP solve state; static RHS before dynamic correction; model metric, not physical clearance",
    "statistics": {
      "count": 1,
      "missing": 0,
      "invalid": 0,
      "mean": 0.2203562,
      "rms": 0.2203562,
      "p95": 0.2203562,
      "minimum": 0.2203562,
      "maximum": 0.2203562
    },
    "inactive_count": 0
  },
  "obstacle.residual": {
    "quantity": "max_positive_unrelaxed_G_u_candidate_minus_h",
    "unit": "m/s",
    "source": "QP solve state; dynamic-corrected RHS; before health gate/integration/filter",
    "statistics": {
      "count": 0,
      "missing": 1,
      "invalid": 0,
      "mean": null,
      "rms": null,
      "p95": null,
      "minimum": null,
      "maximum": null
    },
    "inactive_count": 1
  },
  "obstacle.static_margin": {
    "quantity": "static_cbf_rhs_div_baseline_alpha",
    "unit": "m",
    "source": "QP solve state; static RHS before dynamic correction; model metric, not physical clearance",
    "statistics": {
      "count": 0,
      "missing": 1,
      "invalid": 0,
      "mean": null,
      "rms": null,
      "p95": null,
      "minimum": null,
      "maximum": null
    },
    "inactive_count": 1
  },
  "self_collision.residual": {
    "quantity": "max_positive_unrelaxed_G_u_candidate_minus_h",
    "unit": "m/s",
    "source": "QP solve state; dynamic-corrected RHS; before health gate/integration/filter",
    "statistics": {
      "count": 1,
      "missing": 0,
      "invalid": 0,
      "mean": 0.0,
      "rms": 0.0,
      "p95": 0.0,
      "minimum": 0.0,
      "maximum": 0.0
    },
    "inactive_count": 0
  },
  "self_collision.static_margin": {
    "quantity": "static_cbf_rhs_div_baseline_alpha",
    "unit": "m",
    "source": "QP solve state; static RHS before dynamic correction; model metric, not physical clearance",
    "statistics": {
      "count": 1,
      "missing": 0,
      "invalid": 0,
      "mean": 0.17816501849980926,
      "rms": 0.17816501849980926,
      "p95": 0.17816501849980926,
      "minimum": 0.17816501849980926,
      "maximum": 0.17816501849980926
    },
    "inactive_count": 0
  },
  "singularity.residual": {
    "quantity": "max_positive_unrelaxed_G_u_candidate_minus_h",
    "unit": "model_manipulability/s",
    "source": "QP solve state; dynamic-corrected RHS; before health gate/integration/filter",
    "statistics": {
      "count": 1,
      "missing": 0,
      "invalid": 0,
      "mean": 0.0,
      "rms": 0.0,
      "p95": 0.0,
      "minimum": 0.0,
      "maximum": 0.0
    },
    "inactive_count": 0
  },
  "singularity.static_margin": {
    "quantity": "static_cbf_rhs_div_baseline_alpha",
    "unit": "model_manipulability",
    "source": "QP solve state; static RHS before dynamic correction; model metric, not physical clearance",
    "statistics": {
      "count": 1,
      "missing": 0,
      "invalid": 0,
      "mean": 0.27553186552965975,
      "rms": 0.27553186552965975,
      "p95": 0.27553186552965975,
      "minimum": 0.27553186552965975,
      "maximum": 0.27553186552965975
    },
    "inactive_count": 0
  }
}
```

## 数据与范围限制

- no positive progress beyond floating-point comparison resolution
- projected progress regressed; interval coverage invalid
- observed start differs from predeclared scope start
- declared interval has not been validly completed
