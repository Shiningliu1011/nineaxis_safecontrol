# 自体过滤 bridge 接线：逐源 MoveIt 几何过滤

- Tracker：https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/22
- 日期：2026-09-11；认领：Shiningliu1011。
- 阶段：本地审计、隔离复现与复用接口研究；产品接线未实施，整票 OPEN。
- 起点 HEAD：`3ec19a41aebfa1567d55caa851feed3bf4d9cbee`；保留全部已有未提交文件。
- 用户指令：“wayfinder 继续下一个ticket”。严格前沿已空；依据推荐窗口顺序允许推进独立研究，选择[时间模型交接](7-perception-time-model.md)指向的本票。标定 SSOT 和时间模型原生阻塞均保留，不接管其他已认领票。

## 继承的决定

首版复用 MoveIt Humble ShapeMask 几何包含能力，按每源采集时刻分别过滤，再融合；只删除可信本体点，边界不确定点保留或标记未知。暂不引入阴影射线过滤。过滤几何与 OBB 自碰撞包围几何分工不变。依据[观测契约最终决议](../issues/04-safety-observation.md#resolution-comment已发布为仓库决策记录)和 ADR 0006，不重新请求架构选择。

## 本地证据

1. `perception_bridge.py:209–214,367–369` 只缓存最新 JointState；无历史状态，也未消费它。`_fusion_callback` 调用 feed_camera/feed_lidar 不传 robot_spheres。当前生产 bridge 实际没有机器人几何过滤。
2. `_sensor_callback` 先随机截取超量点、按配置 frame 做变换、裁剪和源体素降采样，然后缓存。消息 frame 与配置 frame 没有一致性检查；采集 TF 失败仍回退 latest/静态矩阵/identity，缺口沿用标定 SSOT 交接。不能在此基础上把 latest JointState 补传给融合器就算接线完成。
3. `FusionEngine.fuse` 先合并并做融合降采样，再将两路 robot_spheres 合并后过滤。球接口即便补接，也不是按每源采集时间隔离的过滤路径。
4. 隔离合成复现：相机 t=1.00 时机器人位于 A、看到外部点 B；LiDAR t=1.05 时机器人位于 B、看到外部点 C。逐源过滤应保留 B、C；旧接口合并掩码后仅保留 C。此例刻意验证不同时间的几何不能交叉删除，不是运动速度或实机工况验收。
5. 隔离顺序复现：同一 3cm 体素内，一个点在半径1cm的本体掩码内，另一个点在2cm外。先体素取首点再过滤，结果为空；先过滤再降采样保留外部点。源预处理目前也先降采样且没有传掩码，因此未来新增过滤不能只放到融合前、源降采样后。
6. URDF 的 base_link、Link1–Link9 各一个 mesh collision；tool0 无 collision。不能宣称实际工具、附件、线缆已覆盖。模型记录必须明确缺失的实体；不能拿 OBB 外包络直接当剔除范围。
7. 本机 `ros-humble-geometric-shapes` 为 `2.3.4-1jammy.20260726.110819`；dpkg 未找到 `ros-humble-moveit-ros-perception`，/opt/ros/humble 无 ShapeMask 头文件。本轮未安装或构建，不能宣称 ShapeMask 已可在当前环境直接编译。

复现资产位于 `.scratch/oscbf-reuse-wayfinder/22/`：`audit_filter.py`、`audit-results.json`、`audit_bridge.py`、`bridge-audit.json`。结果记录相关源码 SHA256。运行：

```bash
python3 .scratch/oscbf-reuse-wayfinder/22/audit_filter.py
python3 .scratch/oscbf-reuse-wayfinder/22/audit_bridge.py
python3 -m pytest portable_oscbf/tests/test_dual_sensor_fusion.py -k 'SelfFilter or robot_spheres' -q
```

两个审计脚本退出0，确认上述缺口；已有过滤测试3通过、29未选中。它们仅覆盖手工传球的单源例子及不传球时保留点，不覆盖真实 bridge、源间时间或近体素外障。没有新增生产测试、运行 ROS 节点或访问硬件。

## 上游复用与关键限制

详见[固定版本上游核验](../research/moveit-shapemask-self-filter.md)：MoveIt2 2.5.9 的 `moveit_point_containment_filter` 是安装并导出的公共共享库，接口头文件为 `moveit/point_containment_filter/shape_mask.h`；所属 `moveit_ros_perception` 标记 BSD，文件为三条款 BSD。优先直接依赖，不因缺少本机安装而复制或 fork 上游。具体安装版本、target/ABI及本机链接仍须构建探针核验。

- ShapeMask 不解释 stamp/frame；变换回调没有时间参数。回调失败仅日志，可能继续使用旧 pose，必须由适配层先验证整帧所有几何变换。
- 仅有 INSIDE/OUTSIDE/CLIP，没有 UNKNOWN 或阴影分类。范围裁剪按输入坐标原点计算，sensor_origin 参数被忽略；空形状集合甚至直接全部 OUTSIDE，因此必须验证形状确实加载。返回 OUTSIDE 不能证明有效空闲。
- geometric_shapes 2.3.4 对 mesh 创建 ConvexMesh，按顶点凸包做包含判断。即使 scale=1、padding=0，凹陷内外部物体仍可能被当成本体；负 padding 也不自动证明删点区域可信。当前模型全部是 mesh，**不能按原始 URDF mask 的 INSIDE 无条件删除**。每个剔除区域需证明在几何/标定/时间误差域内可信，未核验区域保留或未知；这不是允许扩大 OBB 或引入未经批准的影子过滤。
- 直接复用 ShapeMask，比将 Octomap 更新器的 filtered_cloud_topic 接入更符合当前接口：后者仍绑定地图更新，只保留 XYZ且受subsample影响，不能承担本项目扫描时间字段、来源及有效性契约。

这些限制不推翻已定的基础过滤路线，但阻止“安装库后直接删 INSIDE 即完成”的实施方案。若后续发现无法形成可信剔除区域，应回到观测决策重审；本轮不代替用户选择新算法或接受误删。

## 适配契约与实施顺序

推荐沿用现有 Python bridge，增加独立 C++ 过滤适配节点来调用上游库；具体部署封装属于实施细节。先做只读离线输入/输出，不持有命令流发布能力，不把 Octomap 更新器附带输出直接当作本项目观测有效性接口。

1. 每源输入保持原始 PointCloud2 字段、header及采集区间，保留时钟映射、标定ID和源身份。解码、有限值检查之后，在源降采样/随机截断之前完成可信本体分类；容量不足显式报告，不能随机遗漏后报告完整覆盖。
2. 机器人状态按关节名完整映射九轴；保存有时间戳的状态历史。只用覆盖该源采集时刻/扫描区间的有效状态及标定快照，禁止 latest 回退。状态插值、扫描分段及可接受误差界必须随时间模型验证；扫描运动界未知时不把整帧近体点判成可信本体。
3. ShapeHandle 对应 link、collision shape index、collision origin 和模型版本；使用机器人状态的 collision-body transform，不能遗漏 collision origin。每源冻结所有变换，再运行 mask；模型/标定变化采用新版本快照，禁止半帧混用。
4. 推荐在各自传感器坐标中调用范围裁剪与 ShapeMask，以免把 base_link 原点距离误当传感器量程。后续变换到既定 base_link；固定外参来自 SSOT，TF 只是同一记录的派生表达。
5. 输出保留原始采集时间和字段，并附源ID、模型/标定/过滤配置身份、输入/保留/剔除/不确定计数、状态时间与失败原因。过滤后的空点云仍须有一次处理结果；“全本体”“量程裁剪”“输入空帧”“过滤失败”不能合并成安全空场景。
6. 融合只消费各源过滤结果及其有效性；不再将不同时间的掩码合并过滤合并点云。旧球接口只保留为显式测试/兼容对照，不能再次过滤新路径。
7. 未知、错误TF、缺关节、缺模型、状态过期、源重启、容量溢出须进入观测契约和感知健康路径；不在本票创造新的超时/裕度数值。保留有依据的旧观测仅限其有效域，不以过滤失败输出空点云刷新时间。

## 验收与回退

必须覆盖：两源各自不同采集时刻的运动机器人；本体与近外障同体素；URDF collision origin、mesh尺度和附件；错帧/缺关节/乱序/扫描期间运动；ShapeMask变换回调失败；边界、凹陷与紧邻外物；全本体、输入为空及全部裁剪的状态区别；模型/标定更新原子性；故障输出不能沿旧缓存静默继续。

数值报告分别记录本体漏删与外物误删、适用的几何/时间/标定误差域、点数与p50/p95/p99/最大延迟。阈值及目标机长时间实测尚未提供，不把三个旧测试通过当作验收。

可以开始隔离 C++ 构建探针、带版本的输入/输出封装和上述回放。正式接线前需标定 SSOT、时间状态历史及模型可信范围就绪；启用前还须观测与健康契约落实。没有新的架构取舍待用户重选，运行参数和几何可信范围不能自行定案。

回退时停用新适配节点、恢复成对的生产者/消费者及模型/配置快照；旧路径未过滤这一事实必须保留在状态中，不能因回退获得避障准入。本轮仅新增交接、研究和隔离审计资产；未修改控制或自碰撞实现，未提交、部署或启动实机。整票不关闭，地图不追加已解决索引，不自动开始下一票。

## 续研窗口：模型实体与构建前置核验（2026-09-11）

用户指令：“wayfinder 开始ticket#22”。起点 HEAD 为 `84797408ac0cbe344bcb9f06fc70da5ae29dd417`；再次认领原票，原生依赖仍为开放的「标定 SSOT 接线」及「感知时间同步与延迟模型」。本轮保留时间模型交接和陈旧度预算研究的已有未提交改动，只推进本票的独立离线核验。

### 新增可复现证据

- [几何审计脚本](../../../../.scratch/oscbf-reuse-wayfinder/22/geometry-probe/audit.py)与[完整结果](../../../../.scratch/oscbf-reuse-wayfinder/22/geometry-probe/results.json)：读取实际 URDF 的每个 collision mesh，应用显式/default scale，用 trimesh 5.1.0 默认 `process=True` 处理，记录拓扑、体积可用性、局部包围盒及 URDF、各 mesh、脚本 SHA256。运行 `python3 .scratch/oscbf-reuse-wayfinder/22/geometry-probe/audit.py`，退出0。
- 十个网格（base_link、Link1–Link9）均为 `watertight=False`、`is_volume=False`、`is_convex=False`，但绕序一致。**这是此加载器及默认处理下的拓扑诊断，不是物理机器人非封闭或实际误删比例的证明。** 导出接缝、重复面、多组件或建模问题仍需定位；本轮未自动修补、填洞或改变合并容差。
- 因封闭体条件不成立，报告将凸包/实体体积比设为 null，不能拿有符号网格体积推导“误删百分比”。即使修复拓扑，仍须验证模型与实物的一致性以及误差域，才可能作为可信删点依据。
- 十个 collision origin 当前都是零 xyz/rpy，mesh scale 都是默认1；这仅为当前文件事实，后续适配仍必须消费这些字段。`tool0` 无 collision，工具/附件/线缆覆盖仍未证明。
- [CMake 探针](../../../../.scratch/oscbf-reuse-wayfinder/22/build-probe/CMakeLists.txt)及[配置日志](../../../../.scratch/oscbf-reuse-wayfinder/22/build-probe/configure.log)：显式指定 `/opt/ros/humble` 后，`find_package(moveit_ros_perception REQUIRED)` 失败，退出1，找不到包配置。编译器探测成功（GNU 11.4.0）。dpkg 再确认 perception 包未安装、geometric_shapes 为 `2.3.4-1jammy.20260726.110819`。这证明当前前缀的依赖发现门未通过，**没有完成 ShapeMask 编译、链接、ABI或运行测试**，也不代表上游接口不可用。

资产位于本地 scratch，未提交或推送，远端不能据此假定可访问。无需把已有上游核验再做一遍；继续直接依赖公共库，不复制上游几何算法。

### 对实施入口的影响

可继续独立编写离线输入/输出校验、故障回放及依赖准备后的链接探针。正式删点接线仍缺三个具体输入：

1. 与实际机器人对应的可信过滤几何记录：link/collision身份、原始模型及版本、几何有效性证据、可删除区域、工具附件缺口与适用误差域。不能通过本次拓扑失败直接决定改用球、OBB、凸包或自动修复模型。
2. 标定 SSOT 与逐源采集区间对应的机器人状态历史，缺失时明确 unknown/invalid；不能 latest 回退。
3. 可构建的 MoveIt perception 依赖和离线运行证据；安装后依次核验公共target/链接、简单解析形状、完整快照与回调失败、再接本项目已验证模型。

推荐先核查 CAD/STL 导出实体与工具附件清单；没有可信实体的区域继续保留或未知，沿用已定决议，不新增 padding 或容差数值。几何拓扑修复也不能自动授权删点。后续验收须区分“模型有效”“候选包含”“可信剔除”和“观测有效”，分别覆盖边界外障、凹陷、不同源时刻、缺状态和空输出。

本轮是离线审计，不是产品修复验收；未改生产代码、配置、模型或 OBB，未安装包、启动 ROS 或访问硬件。停止使用探针即可回退。整票保持 OPEN，依赖不变，不追加地图已解决索引；本轮不自动启动其他票。
