---
title: 核验 OSCBF 上游能否承接九轴控制与碰撞模型
label: wayfinder:research
status: published-repository-record
assignee: codex-research
parent: ../MAP.md
blocked_by: []
tracker_id: docs-only-parent-issue-1
---

## Question

核验 StanfordASL/oscbf、oscbf_hardware_ws、cbfpy、frax、bubblify 的真实接口、许可证、版本约束、维护信号，以及 1P8R、速度级、5D 工具轴、动态障碍和现有路径跟随的适配缺口。哪些可以直接依赖，哪些需薄适配或最小 fork？不能把论文性能当本机指标。

## Context pointer

本地离线研究草案；GitHub 未登录，未进行远端领取。研究产物见下方；完成状态只在本地有效。

## Resolution comment（已发布为仓库决策记录）

研究事实已核验，结论与证据仅存于 [控制与机器人模型上游核验](../research/control-upstream.md)。CBFpy/QP 已有实质复用；上游 OSCBF 的固定 6D、原 Manipulator 和依赖版本阻止无缝替换现有 5D；frax 为候选，需九轴对照。最终选型留给控制接入决策。

Context branch: `research/oscbf-reuse-control`；commit: `a23f5d7193521801fa04a4d92c6d789e6348d683`。研究问题已回答；兼容安装/九轴测试为后续验证门，未声称完成。未关闭远端 issue。
