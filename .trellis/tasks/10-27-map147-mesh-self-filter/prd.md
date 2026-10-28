# 逐点时间 mesh ray self-filter

来源为地图 #147、实现任务 #105、ADR 0028/0038 与已完成 #108。

## 要求

- 原始 URDF collision mesh、共享 FK、逐点采集时间和反馈历史共同确定每条射线的首个机器人交点。
- 机器人之前的环境返回保留 occupied；可见前缀取得有依据的 free；机器人表面及后方保持 unknown。误差不能支持唯一分类时撤销覆盖准入。
- 输入保留原始点索引、原始时间、monotonic 映射、LiDAR 原点、标定、模型与几何身份和误差来源。
- mesh、标定、时间映射或反馈历史无效时，旧 free 不能继续形成有效场景。
- 模块通过真实占据 update/snapshot 和 CollisionSafety.prepare_scene 验收，ROS 数据适配使用成熟消息库。

## 验收

真实 trimesh 首交点与独立解析距离一致；覆盖正常表面、表面前障碍、表面后遮挡、多个 link、扫描运动、时间与历史缺失、几何变更及旧 free 撤销。实际读取原九轴 URDF/STL，保留当前几何和标定尚未验证的状态。真实 LiDAR 固定回放及跨进程双缓冲验收由 #122 完成。

未授权 commit、push、PR、设备动作或外部写入。
