---
title: 核验双传感器感知与环境距离链的复用方案
label: wayfinder:research
status: published-repository-record
assignee: codex-research
parent: ../MAP.md
blocked_by: []
tracker_id: docs-only-parent-issue-1
---

## Question

核验 Livox Mid-360S 与 Gemini 335L 官方 ROS2 驱动、ROS2 Humble 自体过滤、PCL 最近邻、MoveIt 环境接入、可选 nvblox 的兼容性/许可证/维护证据。即时距离与动态跟踪的职责如何分离？列出观测稀疏、遮挡、时间/TF 误差、距离梯度切换的适用边界；不要把最近点距离等同于安全证明。

## Context pointer

本地离线研究草案；GitHub 未登录，未进行远端领取。研究产物见下方；完成状态只在本地有效。

## Resolution comment（已发布为仓库决策记录）

研究事实已核验，完整答案见 [双传感器感知与环境距离链上游核验](../research/perception-upstream.md)。官方驱动有型号支持依据；自体过滤存在 Humble 工具链兼容门槛；PCL/MoveIt 可复用，nvblox 需锁发行版并核算力。距离观测与动态跟踪应分别定义，最近点不构成真实障碍距离的保守保证。最终表示与过滤选型留给观测契约决策。

Context branch: `research/oscbf-reuse-perception`；commit: `d983b1ff04daa370639d1d2e3d8a0e2233bcc57d`。研究问题已回答，未进行上游构建、传感器实测或端到端验证；未关闭远端 issue。
