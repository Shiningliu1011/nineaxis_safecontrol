# 当前并行实现的独立审查

固定基准与各工作区HEAD均为`9c354d5505e26aa28b3e6fc323c661bd7542b105`。审查包含累计未提交diff及对应冻结清单中的untracked文件。Spec与Standards分别运行，报告与原始失败保留在各票证据目录。

## Spec

- #152：最终三项行为修订复核无发现，18项选择检查及真实独立探针通过。报告为`.scratch/map147-152/review-spec-fixed.md`；后续Standards增量经AST和数值文件身份核验保持原行为。
- #105：第三方解码异常边界的主会话修订复核无发现；三种真实解码失败均撤销旧free、保留occupied并继续抛出原始错误。报告为`.scratch/map147-105/review-repair-parent.md`。
- #101：fix3独立13项公开边界检查通过，无新增发现；全部/单个固定模型成员遗漏、group与旧静态group遗漏均拒绝，合法合并、模型身份与容量失败符合要求。报告为`.scratch/map147-101/review-spec-fixed3.md`。

## Standards

- #152：共享关节身份及共同保留索引的增量检查无发现，三项真实选择检查通过，42来源文件身份在检查前后相同。报告为`.scratch/map147-152/review-standards-repair-parent.md`。
- #105：修订增量仅涉及异常边界、真实解码测试和说明，其余14份交付与四份依赖身份相同；复核无新增问题。原报告与主会话增量分别为`.scratch/map147-105/review-standards.md`及`review-repair-parent.md`。
- #101：fix3只读增量复核无发现，22来源与25证据身份相同；沿用原四项实际通过证据。报告为`.scratch/map147-101/review-standards-fixed3.md`。

## 集成与证据范围

#152与#105已完成main集成、完整功能覆盖和安装核验，主会话独立身份检查为`.scratch/map147-decisions/parent-integration-verification.json`；两票已按completed关闭。controller四个P-core与modular八个P-core分别通过原计时门槛，全部CPU条件仍失败。完整项目周期尚未验证。

#101的fix3来源清单为`.scratch/map147-101/review-fix3-handoff.json`，SHA256为`4fee0f00584babac2628b516c16b96fa75fecdcede05dde8afeac82e81aa0206`。main最终集成清单为`.scratch/map147-101/integration/final-files.json`，SHA256为`f73ac2871556604e70ab70c01f4af739de0bec7d1aebe7684ff41b2781979d42`。456项相关消费者、30项独立组合检查、43项实际安装接口通过，36份实际安装文件与main一致。主会话独立核验23份main文件、43份证据、76份保留身份及全部原始JUnit；#101已按completed关闭，完成评论6446396500。原8.349961508s期限失败原因未确定，原日志继续保留。

规范提升审阅：本轮已有确认的决定继续由对应文档管理；本轮不新增规范、CONTEXT或ADR。此前并行交付的提交与推送授权属于该批工作，本次路径参考修复的授权与验证见下节。后续任务保持未启动状态。

## 8089998 路径参考修复验收

本次修复以已同步的`main`提交`930b582a447d29af8e289bdd24c6ae5ed255abe6`为起点，开始时工作区干净。用户已明确授权将两项 P2 的修复、测试和相关文档提交到 `main`，并推送到 `origin/main`，完成同步后停止。

1. 已完成连续参考自动发现修复。`portable_oscbf/work/ik_data_loader.py`的`_find_continuous_reference()`与`attach_continuous_reference()`共用完整配置身份检查，比较当前模型、变换、两次投影的轴向、中心和半径。真实生成默认配置与`ee_center=[0.0,0.344,1.387]`两个版本，各自14992个来源点验证通过；相同来源与半径下，自动发现选择对应变换的版本，显式连接保持身份拒绝，选定产物继续执行完整验证。
2. 已完成圆柱姿态重新设置修复。`set_surface_normal_orientation()`从首次投影完成的位置与运动参考重新构建当前圆柱，维护两次投影表达。production/S0 的拟合及显式参数、重复设置和`surface_normal→fixed→surface_normal`共八种组合连续重新设置三次后，连续身份、弧长及全部14992个样本的运动参考保持一致。改变轴向、中心和半径后恢复至production/S0，与相同参数的新加载结果一致。

可选维护建议：`nineaxis_model.py`与`butterfly_reference.py`分别保存相同的源工具轴转换矩阵；当前数值一致，可采用共同来源。

实际验证：真实资产及连续参考组合55项通过（184.18 s），公共路径消费者26项通过（28.53 s），两组退出0，无失败、错误、跳过或未选中用例。检查覆盖production/S0、NumPy/JAX、安装资源、错误模型、缺少匹配配置、选定产物损坏及姿态重新设置。Python编译、AST、JUnit和生成版本来源核验通过，既有连续资源保持未改动。完整命令与TDD记录见`../implement.md`。本次没有重新运行全量测试、性能矩阵或设备测试。

原始审查证据保留在设备`lsn`的`.scratch/pr-review-20261028/`。本次修复证据独立保存在`.scratch/reference-p2-fix-20261028/`，包括TDD日志、两组最终回归日志与JUnit，以及`verification.json`中的生成版本身份和核验结果。使用说明更新于`docs/butterfly_reference.md`。两项修复与相关验收完成，修复、测试和相关文档共同纳入 `main` 提交与 `origin/main` 同步范围，同步结果通过实际 Git 分支身份核验。其余地图任务保持原状态。

## 2026-10-28 参数检查与显式参考来源验收

本次基点为`8089998d9c937694b39ef5a0653a9d58ccb6ef33`。用户授权完成两项 P2 的修复、真实数据回归和文档更新，并将本次改动提交到 `main`、推送到 `origin/main`，完成同步后停止。开始时工作区干净，其余地图任务保持原状态。

1. 参数检查及圆柱拟合在当前轨迹状态更新前完成。轴向与中心要求三个有限分量，轴向非零，指定半径有限且为正；拟合采用首次投影点。十种无效参数的真实 S0 回归中，全部14992个位置、旋转、速度、加速度、角速度和 jerk，以及连续身份和几何保持一致。
2. 成功显式连接后保存参考目录，姿态重复设置及经过`fixed`恢复后重新验证该来源。显式目录在自动采用其他圆柱版本时保留，恢复原参数后重新采用原显式版本。配置身份、来源 SHA256、完整文件与连续表达仍共同验证；来源损坏或 manifest 缺失时拒绝加载。

Standards 独立静态复核通过，未发现新的明确规范违反。Spec 独立成功探针的八项真实检查通过，覆盖拟合及显式参数、两种姿态恢复、S0自动选择后恢复、无效参数保持状态、来源 SHA256 损坏与 manifest 缺失拒绝。独立结果为`.scratch/reference-state-fix-20261028/review-spec/state-fix-results.json`，未重复运行主会话的pytest。

主会话真实资产与连续 NumPy/JAX 回归61项通过（219.59 s），公共路径消费者26项通过（29.01 s），均退出0，无失败、错误、跳过或未选中用例。Python编译、AST、JUnit、来源验证及`git diff --check`通过，既有连续资产保持未改动。完整命令及TDD记录见`../implement.md`，全部新证据保存在`.scratch/reference-state-fix-20261028/`，代码及测试身份保存在`verification.json`。

使用说明已更新`docs/butterfly_reference.md`。本次没有运行全量测试、性能矩阵或设备测试。两项修复与相关验收完成，修复、真实数据回归测试和相关文档共同纳入 `main` 提交与 `origin/main` 同步范围，完成同步并核验分支身份后停止。
