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

规范提升审阅：本轮已有确认的决定继续由对应文档管理；本轮不新增规范、CONTEXT或ADR。用户已授权将本轮工作提交到本地main并推送origin/main。后续任务保持未启动状态。

## 2026-10-28工作区审查待处理事项

本次审查冻结main上139份未提交与未跟踪文件，基准与HEAD均为`9c354d5505e26aa28b3e6fc323c661bd7542b105`。Standards未发现明确违反；Spec与主会话实际探针确认以下两个P2问题，当前同步保留其行为。

1. `portable_oscbf/work/ik_data_loader.py`的`_find_continuous_reference()`仅按来源MAT和`projection_radii_m`选择版本，随后立即执行完整身份验证。相同半径、不同位置变换的文件并存时，排在前面的版本会抛出异常，后面的匹配版本无法被发现。真实生成默认配置与`ee_center=[0.0,0.344,1.387]`两个版本，各自14992个来源点验证通过且半径一致；直接连接后者成功，自动发现抛出变换或圆柱身份不匹配。应在选择候选时完整比较当前变换、投影和模型身份。
2. 同一文件的`_snap_path_onto_cylinder()`每次向`projection_stages`追加阶段，`attach_continuous_reference()`要求恰好两个阶段。重复设置相同`surface_normal`，或`surface_normal→fixed→surface_normal`，均使阶段数从2变为3。全部14992个位置的最大变化仅`2.220446049250313e-16 m`，`path_geometry(require_continuous=True)`随后拒绝已有有效参考。重新设置姿态需要维护当前有效的两次投影表达。

可选维护建议：`nineaxis_model.py`与`butterfly_reference.py`分别保存相同的源工具轴转换矩阵；当前数值一致，可采用共同来源。

实际验证：`python -m pytest -q tests/test_butterfly_reference_assets.py --basetemp=.scratch/pr-review-20261028/assets-pytest-data`退出0，4项通过，耗时26.68s。静态检查包括44份Python AST、配置格式、依赖声明、连续参考产物及模型来源身份；139份文件SHA256与HEAD在接手记录更新前全部一致。本次没有重新运行全量、性能矩阵或设备测试。

原始证据保存在设备`lsn`的`.scratch/pr-review-20261028/`：`check_reference_discovery.py`与`reference-discovery.log`记录首项复现；`spec/run_orientation_probe.sh`与`spec/orientation-reset.json`记录第二项复现；`assets-pytest.log`保存四项现有测试结果；`scope.json`保存文件身份。审查没有修改生产代码与原有测试。远程同步包括源码、连续参考资产、测试、规格和Trellis接手记录，原始审查目录继续按现有规则保留本地。
