# 执行与验收

- [x] 用真实 `data/nurbs/ik_input.mat` 记录修改前的完整轨迹、采样轨迹和时间序列到 `.scratch/i17/`；首目标由轨迹一致性测试检查。
- [x] 更新轨迹入口、ROS 参数、配置及相关测试。
- [x] 更新关节配置加载函数，使运行时核对项目关节顺序及限幅来源。
- [x] 更新相关文档并检查仓库检索结果。
- [x] 用相同输入逐值比较修改前后的轨迹数据。
- [x] 运行相关测试、`bash run_all_tests.sh` 和适用的项目检查；记录命令与结果。
- [x] 完成 Trellis 检查与票尾验收记录。

当前任务只处理 GitHub issue 128。已有用户改动保持原状。

## 本次验证

- 日期：2026-09-24；基准提交：`633b963c61725e2bf3582f59c19426150f19fdcf`；工作区有本次改动及用户已有的票据流程文档改动。
- 输入：`data/nurbs/ik_input.mat`，SHA-256 `79868b03afd68f9d4cca1cba568320f0f48efd355d020a9372f2037549c3293f`。
- `python3 .scratch/i17/trajectory_compare.py capture`：退出码 0；修改前数据保存于已忽略的 `.scratch/i17/trajectory_baseline.npz`。
- `python3 .scratch/i17/trajectory_compare.py compare`：退出码 0；完整路径 14992 点、采样路径 64 点、时间序列 64 项、控制器路径 14984 点均逐值相同。
- `python3 -m pytest tests/test_robot_config_sources.py tests/test_trajectory_transform_unity.py -q`：9 项通过。
- `python3 portable_oscbf/scripts/generate_kinematics_data.py --check`：退出码 0。
- `bash run_all_tests.sh`：退出码 0；主包 547 项通过、1 项跳过；内核 178 项通过、34 项跳过；插件 4 项通过。
- `bash scripts/agent_check.sh`：退出码 0；纯逻辑导入、Python 编译、shell 语法、Git 空白及 69 项纯逻辑测试通过。
- 项目源码与任务记录检索旧参数名称：无结果；`portable_oscbf/config/` 中没有手写的 `joint_limits` 段。
- 规范审查：现有项目规范已覆盖本次使用的关节顺序、URDF 生成数据和共享轨迹入口，无新增规范内容。
- 2026-09-25 用仓库内已忽略的临时目录复核新增测试与轨迹测试：9 项通过；Git 跟踪文件检索旧参数名称无结果；`git diff --check main` 通过。
- 票尾验收记录：[实现票评论](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/128#issuecomment-5829242255)；实现票已关闭，统一执行地图的决定索引已更新。
