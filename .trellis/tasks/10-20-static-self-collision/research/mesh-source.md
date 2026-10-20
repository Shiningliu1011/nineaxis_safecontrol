# 现有闭合几何

- 来源清单：`portable_oscbf/config/collision_geometry_sources.json`。
- 派生 URDF：`models/ninezzhou/urdf/ninezzhou_collision.urdf`。
- 来源记录：`models/ninezzhou/collision_meshes/provenance.json`。
- 十个闭合 STL 均为已有生成结果，来源记录保存原 mesh 与闭合 mesh 的 SHA256。
  原生成器使用凸包包围原有模型，记录包含检查结果。当前实验直接使用这些文件。
- 十个 collision origin 均为零，单位为 m，未指定额外 mesh scale。
- 原生成器 `generate_closed_collision_meshes.py --check` 重新计算并逐文件核查，
  此参数只读源文件与既有结果。
- 原生扩展调用 FCL 0.7.0，查询容差显式为 1e-12 m，已有 Convex 构造入口。
- ros/rosdistro 的 `rosdep/base.yaml` 已核实 `libfcl-dev`、`pybind11-dev`
  在 Ubuntu 上的安装映射，下载副本位于忽略的 `.scratch/static-experiments/`。
