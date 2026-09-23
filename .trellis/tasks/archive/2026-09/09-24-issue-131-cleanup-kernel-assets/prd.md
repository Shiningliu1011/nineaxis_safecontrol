# I16 清理内核无读取方的配置与数据

## 来源与目标

来源：[[I16] 清理内核无读取方的配置与数据](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/131)，属于[速度级 OSCBF 与 CollisionSafety 统一执行地图（仅激光雷达感知）](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/133)。

删除内核目录中没有读取方的配置与数据，保留当前轨迹读取能力，并确保安装清单和安装产物准确。

## 需求

- 删除 `portable_oscbf/config/` 下的 `controller_params.yaml`、`fcl_params.yaml`、`ompl_params.yaml` 与 `perception_runtime.yaml`。
- 删除 `portable_oscbf/data/` 下的 `nurbs_blocks.mat`、`realtime_interpolation_results.mat`、`workspace_ik_input.mat`、`workspace_nurbs_blocks.mat` 与 `workspace_realtime_results.mat`。
- 保留内核回归测试读取的 `portable_oscbf/data/ik_input.mat`，并确保 `setup.py` 将该文件安装到内核数据目录。生产轨迹 `data/nurbs/ik_input.mat` 的加载保持有效。
- 更新说明内核文件清单、配置用途与安装位置的当前文档。
- 保留当前 sim、shadow、live 硬件安全边界。

## 验收条件

- [x] 指定的九个文件从跟踪工作区移除，在安装产物中也不存在；保留的内核数据文件与安装清单一致。
- [x] 内核回归轨迹与生产轨迹均可读取。
- [x] 内核测试、`bash build_aeb_moveit.sh` 与 `bash run_all_tests.sh` 均通过。
- [x] 当前文档的文件清单与配置说明反映实际目录，用户已有修改保持原样。
