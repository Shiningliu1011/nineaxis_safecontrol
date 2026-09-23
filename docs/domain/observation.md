## 障碍观测

**即时障碍观测**:
本项目中，在给定采集时刻、坐标与有效范围内对环境障碍几何的观测；其存在不依赖对象身份关联或速度估计成功。
_Avoid_: 已跟踪障碍（作为全部在线障碍的统称）

**局部占据场景**:
LiDAR 射线在机器人附近更新的 `occupied`、`free`、`unknown` 空间状态；占据 voxel
生成环境碰撞 support points，自由与未知状态用于覆盖准入。
_Avoid_: 单帧点集、把无返回区域称为自由空间

**占据证据规则**:
LiDAR hit 立即建立 occupied，连续且时间有效的 free 射线才能清除 occupied；陈旧占据
转为 unknown，时间经过本身不能产生 free。
_Avoid_: 最新射线直接覆盖、超时后删除障碍

**运动跟踪层**:
局部占据场景之上的可选运动估计；可信 track 为关联 support points 提供速度与 barrier
时间项，不能创建或删除占据证据。
_Avoid_: 用 track 列表代替占据场景、把未跟踪障碍称为空闲

**Support point 米制范围（rho_m）**:
以 support point 为中心、覆盖 occupied voxel 体积与全部空间误差的球形范围；它按机器人
ellipsoid 的最短半轴转换为 pair-specific `scale_margin`。公开配置与诊断使用 `mm`，
`rho_m` 是 collision facade 转换后的 JAX 内部量。
_Avoid_: 零体积点、全局固定 scale_margin

**保守 support 合并**:
用一个新的 `support_point + rho_m` 完整覆盖一组 occupied support；用于满足固定 JAX
容量，无法证明覆盖时场景进入 `OVER_CAPACITY`。
_Avoid_: 截断点云、删除较远 occupied voxel

**固定环境模型**:
经校验的工作台、夹具、地面等已知环境几何，以版本化 `support_point + rho_mm` 进入
`CollisionScene` 并与 LiDAR occupied 取并集；环境变化后需重新校验。
_Avoid_: 静态跟踪结果（指固定环境模型时）

**必要空间覆盖**:
对当前动作及停车所需空间具有仍有效的环境信息，其范围与可信条件由观测契约确定；无返回点不等于具有覆盖。
_Avoid_: 点云非空（作为覆盖充分的同义词）

**环境距离场**:
环境障碍几何的一种固定形状表示：按基准系的固定栅格给出各位形到最近被观测占据位置的距离，机器人侧的固定查询点据此取得与环境的距离；查询点不在栅格覆盖内时按不安全处理。
_Avoid_: ESDF、静态地图（作为该距离场的同义词）

**自体过滤**:
从 LiDAR 射线中识别机器人自身表面返回的处理；使用 point 采集时间对应的关节状态与
机器人 mesh 预计首次交点，机器人侧的组合 ellipsoid 不参与删除环境点。
_Avoid_: 机器人自滤、机械臂点剔除

**歧义观测（ambiguous observation）**:
受测量、标定或时间误差影响，无法明确判定为 robot self 或 environment occupied 的
LiDAR 返回；对应空间不能取得覆盖准入。
_Avoid_: 直接删除、直接标记为 free
