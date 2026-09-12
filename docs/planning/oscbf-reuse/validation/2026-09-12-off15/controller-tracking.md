# 路径跟踪评价报告

- 任务质量：不通过；在线门控：证据不足
- 全部所需判据：不通过（仅限下述证据范围，不授予实机准入）
- 证据：model；测量边界：kernel_candidate
- 运行：oscbf-runtime-20260912T135540.891276Z-d5d79979247448638029c7d248eea552；工况：configured path; obstacles=False
- 测量定义：post-integration model q_next and command reference; QP rows at solve input; before command filter; no execution feedback；时间基准：perf_counter sample start; step_once includes kernel and host diagnostics
- 模型 / 配置 / 路径 / 数据身份：/tmp/pytest-of-lsn/pytest-6/oscbf_m100/runtime_snapshots/oscbf-runtime-20260912T135540.891276Z-d5d79979247448638029c7d248eea552.json#software / /tmp/pytest-of-lsn/pytest-6/oscbf_m100/runtime_snapshots/oscbf-runtime-20260912T135540.891276Z-d5d79979247448638029c7d248eea552.json / sha256:9527603c32cf1dde6d96f714aa44345ad21f7c1d8804db6597f84769e92c38b3 / /tmp/pytest-of-lsn/pytest-6/oscbf_m100/runtime_snapshots/oscbf-runtime-20260912T135540.891276Z-d5d79979247448638029c7d248eea552.json#kernel_step_sequence
- 总样本：5；终止原因：未终止/未声明
- 样本记录 SHA-256：47a153b31d97ecb8db79891775427b4b4175a7aa8e5b4b2b87d087aaae112e97
- 声明范围：EvaluationScope(total_length_m=1.8167634936816308, start_m=0.0, end_m=1.8167634936816308)；初始/最终投影弧长：1.17792 / 0.0264249 m
- 绝对弧长比：0.014545；本次区间覆盖比：0
- 本次区间完成：False；整条路径覆盖：False；全部所需判据下整条路径验证：False
- 采样耗时：0.04 s；计算 deadline：20 ms；超期率：未测
- 未采集边界：filtered_command, simulated_state, real_feedback

## 误差及诊断统计

统计按样本等权；部分有效样本仍可供诊断，缺失/无效样本阻止对应指标通过。

| 指标（单位见定义） | 有效/缺失/无效 | 均值 | RMS | p95 | 最小 | 最大 |
|---|---|---|---|---|---|---|
| actual_tangent_speed_m_s | 5/0/0 | -0.0133107 | 0.0133107 | -0.0133004 | -0.013351 | -0.0133004 |
| cross_track_m | 5/0/0 | 0.525845 | 0.525845 | 0.525847 | 0.525845 | 0.525847 |
| delta_slack | 5/0/0 | 6.31256e-11 | 6.31266e-11 | 6.35914e-11 | 6.26362e-11 | 6.36479e-11 |
| feedrate_m_s | 5/0/0 | 0 | 0 | 0 | 0 | 0 |
| legacy_orientation_error | 5/0/0 | 1 | 1 | 1 | 1 | 1 |
| obstacle_clearance_m | 0/5/0 | 未测 | 未测 | 未测 | 未测 | 未测 |
| obstacle_margin_m | 0/5/0 | 未测 | 未测 | 未测 | 未测 | 未测 |
| online_cross_track_m | 5/0/0 | 0.535198 | 0.535198 | 0.535198 | 0.535198 | 0.535198 |
| pos_error_m | 5/0/0 | 0.744134 | 0.744134 | 0.744135 | 0.744134 | 0.744135 |
| qp_primal_residual | 5/0/0 | 0 | 0 | 0 | 0 | 0 |
| source_time_s | 5/0/0 | 0 | 0 | 0 | 0 | 0 |
| step_latency_ms | 0/5/0 | 未测 | 未测 | 未测 | 未测 | 未测 |
| tool_axis_error_rad | 5/0/0 | 1.57093 | 1.57093 | 1.57095 | 1.5709 | 1.57095 |

## 逐项判据与来源

| 指标 | 类别 | 阈值/单位 | 状态 | 测量值 | 结论 | 来源/原因 |
|---|---|---|---|---|---|---|
| cross_track_m.rms | task | <= 0.00015 m | accepted | 0.525845 | 不通过 | GitHub #14 resolution 2026-09-04; complete samples compared with sourced threshold |
| cross_track_m.p95 | task | <= 0.0005 m | accepted | 0.525847 | 不通过 | GitHub #14 resolution 2026-09-04; complete samples compared with sourced threshold |
| cross_track_m.maximum | task | <= 0.002 m | accepted | 0.525847 | 不通过 | GitHub #14 resolution 2026-09-04; complete samples compared with sourced threshold |
| tool_axis_error_rad.rms | task | <= 0.000872665 rad | accepted | 1.57093 | 不通过 | GitHub #14 resolution 2026-09-04; OFF-15 user decision: true axis angle 2026-09-12; complete samples compared with sourced threshold |
| tool_axis_error_rad.maximum | task | <= 0.00872665 rad | accepted | 1.57095 | 不通过 | GitHub #14 resolution 2026-09-04; OFF-15 user decision: true axis angle 2026-09-12; complete samples compared with sourced threshold |
| qp_success_rate | task | >= 0.999 1 | accepted | 1 | 通过 | GitHub #14 resolution 2026-09-04; complete samples compared with sourced threshold |
| obstacle_clearance_nonnegative | safety | >= 0 m | accepted | 未测 | 证据不足 | GitHub #14 resolution 2026-09-04; nonnegative clearance alone does not prove non-overlap; required samples missing or invalid |
| obstacle_clearance_m.minimum | safety | >= 0.03 m | provisional | 未测 | 证据不足 | GitHub #14 resolution 2026-09-04; geometry/scope pending #8/#18; threshold not accepted for this scope |
| deadline_miss_rate | timing | <= 0.01 1 | provisional | 未测 | 证据不足 | GitHub #14 resolution 2026-09-04; deadline must match current measured boundary/config; threshold not accepted for this scope |

## 在线门控与约束

- QP：success=1，失败=0，缺失/无效=0
- 准入：{'unmeasured': 5}；重叠：{'unmeasured': 5}
- 状态事件：{}；限制因素：{4: 5}
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
      "missing": 5,
      "invalid": 0,
      "mean": null,
      "rms": null,
      "p95": null,
      "minimum": null,
      "maximum": null
    },
    "inactive_count": 5
  },
  "esdf.static_margin": {
    "quantity": "static_cbf_rhs_div_baseline_alpha",
    "unit": "m",
    "source": "QP solve state; static RHS before dynamic correction; model metric, not physical clearance",
    "statistics": {
      "count": 0,
      "missing": 5,
      "invalid": 0,
      "mean": null,
      "rms": null,
      "p95": null,
      "minimum": null,
      "maximum": null
    },
    "inactive_count": 5
  },
  "joint_angular.residual": {
    "quantity": "max_positive_unrelaxed_G_u_candidate_minus_h",
    "unit": "rad/s",
    "source": "QP solve state; dynamic-corrected RHS; before health gate/integration/filter",
    "statistics": {
      "count": 5,
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
      "count": 5,
      "missing": 0,
      "invalid": 0,
      "mean": 0.4306072445556641,
      "rms": 0.4306072445556641,
      "p95": 0.4306072445556641,
      "minimum": 0.4306072445556641,
      "maximum": 0.4306072445556641
    },
    "inactive_count": 0
  },
  "joint_linear.residual": {
    "quantity": "max_positive_unrelaxed_G_u_candidate_minus_h",
    "unit": "m/s",
    "source": "QP solve state; dynamic-corrected RHS; before health gate/integration/filter",
    "statistics": {
      "count": 5,
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
      "count": 5,
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
      "missing": 5,
      "invalid": 0,
      "mean": null,
      "rms": null,
      "p95": null,
      "minimum": null,
      "maximum": null
    },
    "inactive_count": 5
  },
  "obstacle.static_margin": {
    "quantity": "static_cbf_rhs_div_baseline_alpha",
    "unit": "m",
    "source": "QP solve state; static RHS before dynamic correction; model metric, not physical clearance",
    "statistics": {
      "count": 0,
      "missing": 5,
      "invalid": 0,
      "mean": null,
      "rms": null,
      "p95": null,
      "minimum": null,
      "maximum": null
    },
    "inactive_count": 5
  },
  "self_collision.residual": {
    "quantity": "max_positive_unrelaxed_G_u_candidate_minus_h",
    "unit": "m/s",
    "source": "QP solve state; dynamic-corrected RHS; before health gate/integration/filter",
    "statistics": {
      "count": 5,
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
      "count": 5,
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
      "count": 5,
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
      "count": 5,
      "missing": 0,
      "invalid": 0,
      "mean": 0.27553195322861124,
      "rms": 0.27553195322861124,
      "p95": 0.27553195322861124,
      "minimum": 0.27553195322861124,
      "maximum": 0.27553195322861124
    },
    "inactive_count": 0
  }
}
```

## 数据与范围限制

- projected progress regressed; interval coverage invalid
- observed start differs from predeclared scope start
- declared interval has not been validly completed
