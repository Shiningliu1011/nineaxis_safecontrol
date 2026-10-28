# 来源与能力边界

已全文读取仓库外探索资料 `/home/lsn/robot/map147-exploration/self-filter-evidence.md` 及其真实探针结果。当前原 URDF SHA-256 为 `766ea5870c41f110d7d57b46e6b31153335f45358faf15e641d465b3b31587a3`，10 个原始 STL 共 696330 个三角面。共享 FK 的 J1 50 mm 平移独立核验通过。

原 mesh 存在边界、非流形连接及部分零面积三角面；当前 LiDAR 标定 admission_ready=false。上述事实保持独立诊断，不能由模块解析测试转为物理准入。

trimesh 5.1.0 的实际 ray_triangle.RayMeshIntersector 源码已读取；真实 ray 依赖 rtree 1.4.1，显式声明并安装。使用 urdf_parser_py 解析 URDF、sensor_msgs_py 解析 PointCloud2；不编写格式或 ray parser。

#108 来源为主仓库未提交集成，基准 ref 本身不含该内容。所需文件通过文件补丁补充，来源 hash 另存 `.scratch/map147-105/dependency-source.json`，原始清单为 `.scratch/map147-108/integration/final-files.json`。完成后只把本任务增量交给 merger。
