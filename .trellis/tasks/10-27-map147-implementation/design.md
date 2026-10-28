# 任务与集成边界

GitHub原生父子关系与blocking为任务图来源。`docs/planning/nineaxis-full-project/dependencies.md`保存入口，决定正文与确认记录分别保存。主会话管理任务图、已确认需求及跨模块接口，实现agent在各自工作区直接实现，merger独立将通过审查的文件集成至main工作区。

本次 main 基点为 `8d5e938d4fe0185e07f17bc2359881157967baac`，开始时工作区干净。已完成场景、逐点过滤、动态 support、路径与模型能力均以当前文件为依据。场景服务在独立工作区 `codex/map147-scene-server` 实现，通过审查后使用文件编辑工具集成至 main；本次代码、测试与相关文档按用户授权提交到 `main` 并推送到 `origin/main`。

每个实现子任务保留自己的Trellis规划、TDD边界和验收，主会话读取冻结清单后分别组织Spec与Standards审查。修订只重新检查受影响范围，集成后检查实际main消费者和安装副本。

M05/M06/M08/M09/M10/M11/M12及Runtime的写入归属遵守已经确认的共同数据与执行记录。单位边界采用mm/°展示与SI计算，物理时间int64 ns与host耗时分别保存。没有现场参数时不生成物理能力证据。
