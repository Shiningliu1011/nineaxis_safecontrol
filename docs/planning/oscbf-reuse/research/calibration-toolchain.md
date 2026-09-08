# 标定工具链复用与 AX=YB 方向研究

研究日期：2026-09-08。服务于[标定工具链复用 + AX=YB](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/25)。这是选型证据与实施边界，不是实物标定完成记录。已阅读 CONTEXT、ADR 0003/0004；Gemini335L、LivoxMid360S 仅是本地设备声明，未核验实物型号、驱动字段或标定精度。

## 建议

建议将作者版 FAST-Calib2 作为优先验证候选，离线求 LiDAR→camera optical 外参，OpenCV `calibrateRobotWorldHandEye` 离线求固定外置相机→base_link。本项目只需数据导出、方向明确的结果适配和 provenance；无需为一次性离线求解维护 FAST-Calib 的 ROS 2 移植。先在隔离的 ROS 1 工具环境验证上游样例和自身数据，再锁定环境；目前不能称 Ubuntu 22.04/Humble 已验证可用。

LiDAR-camera 使用上游反光环板；末端手眼使用独立、轻型、刚性视觉板。两阶段只需相同 camera optical 坐标及稳定内参，不要求同一实体标定板。相机和 LiDAR 固定安装关系改变后必须重标相关外参。以上是本项目工程建议，不是上游已替本设备完成的验证。

## 上游身份、版本与兼容性

| 工具 | 本次核验版本 | 已核验事实 | 用途 |
|---|---|---|---|
| [hku-mars/FAST-Calib](https://github.com/hku-mars/FAST-Calib) | `1018ecfdf9deda51b91a8a11bd11972a0b159008`，2026-04-19 | catkin/roscpp、ROS 1 bag；GPL v2 | 原版圆孔板方案，备选 |
| [xuankuzcr/FAST-Calib2](https://github.com/xuankuzcr/FAST-Calib2) | `5cacbc2291eb063e4dfe972cb36df9a4bc2cbf6a`，2026-07-16 | catkin/roscpp、ROS 1 bag；GPL v2 | 首选反光环板方案 |
| [OpenCV 4.5.4](https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html) | 4.5.4 API/source tag | 含 Shah/Li robot-world-hand-eye 求解器；Apache 2.0 | 本机已有库，离线调用 |

提交号由浅克隆后 `git rev-parse HEAD`、`git log -1` 实际读取；不是根据搜索摘要猜测。FAST-Calib2 在原作者 Chunran Zheng 个人账户，作者主页直接列出此工具；不要写成不存在的 `hku-mars/FAST-Calib2`。[作者主页](https://zhengchunran.com/)、[FAST-Calib 提交](https://github.com/hku-mars/FAST-Calib/commit/1018ecfdf9deda51b91a8a11bd11972a0b159008)、[FAST-Calib2 提交](https://github.com/xuankuzcr/FAST-Calib2/commit/5cacbc2291eb063e4dfe972cb36df9a4bc2cbf6a)。

两者 README 写 PCL≥1.8、OpenCV≥4.0，但实际 CMake 要求 PCL 1.10，并依赖 catkin、roscpp；不是原生 Humble/ament 包。未发现所检查官方文档承诺 Ubuntu 22.04/Humble 支持，亦未在本研究构建或运行 FAST-Calib。不能把“依赖在机器上存在”当兼容性证据。[原版构建文件](https://github.com/hku-mars/FAST-Calib/blob/1018ecfdf9deda51b91a8a11bd11972a0b159008/CMakeLists.txt)、[第二版构建文件](https://github.com/xuankuzcr/FAST-Calib2/blob/5cacbc2291eb063e4dfe972cb36df9a4bc2cbf6a/CMakeLists.txt)。

建议把上游工具及其许可证独立保存，不把求解代码复制进运行节点。许可证记录依据为两仓库 LICENSE 的 GPL v2 和 OpenCV 4.5.4 LICENSE 的 Apache 2.0；本研究不扩展为分发法律判断。[FAST-Calib LICENSE](https://github.com/hku-mars/FAST-Calib/blob/1018ecfdf9deda51b91a8a11bd11972a0b159008/LICENSE)、[FAST-Calib2 LICENSE](https://github.com/xuankuzcr/FAST-Calib2/blob/5cacbc2291eb063e4dfe972cb36df9a4bc2cbf6a/LICENSE)、[OpenCV LICENSE](https://github.com/opencv/opencv/blob/4.5.4/LICENSE)。

## 板、输入与输出

FAST-Calib2 上游宣称改善大光斑雷达的圆孔边缘误差，列举 Mid360 等设备；这不等于已经验证本地声明的 Mid360S。它要求四反光环与四视觉标记，PVC 板厚至少 1 cm，反光膜为 3M engineering grade。每个静态场景需点云 bag 与对应图像，至少三个不同摆放场景才能按其流程做多场景联合标定。[上游 README](https://github.com/xuankuzcr/FAST-Calib2/blob/5cacbc2291eb063e4dfe972cb36df9a4bc2cbf6a/README.md)。

默认配置的板宽 1.4 m、高 1.0 m，marker 边长 0.20 m，环中心距 0.5/0.4 m，环中心线半径 0.12 m、半带宽 0.025 m；这些是模板参数，必须与制作实测相符，不能直接认为适合机器人末端负载。图像分辨率必须匹配内参对应分辨率，镜头畸变系数须匹配输入是否已去畸变，不能把示例相机内参当 Gemini 参数。[配置](https://github.com/xuankuzcr/FAST-Calib2/blob/5cacbc2291eb063e4dfe972cb36df9a4bc2cbf6a/config/qr_params.yaml)、[分辨率校验](https://github.com/xuankuzcr/FAST-Calib2/blob/5cacbc2291eb063e4dfe972cb36df9a4bc2cbf6a/include/common_lib.h#L175)。

数据转换必须保留浮点 XYZ（米）与强度。上游 ROS 1 reader 支持 `livox_ros_driver/CustomMsg` 和 `sensor_msgs/PointCloud2`；后者按 float 读 intensity/reflectivity，有 `ring` 就判机械雷达、无则判固态，ring 按 uint16 读取。因此 ROS 2 Livox 数据不能只改消息名；需核验类型并显式规范为符合该读取器的输入，避免 Mid360 因人为新增 ring 走错算法。bag 中所有帧会累积，板与传感器必须保持静止。[数据读取源码](https://github.com/xuankuzcr/FAST-Calib2/blob/5cacbc2291eb063e4dfe972cb36df9a4bc2cbf6a/src/data_preprocess.hpp#L75)。

两版本 SVD 都以 LiDAR 中心为 source、camera 中心为 target，输出 `Rcl/Pcl` 即 `camera_T_lidar`；按列向量有 `p_camera = Rcl p_lidar + Pcl`。颜色投影也沿相同方向。camera 在这里是与图像内参/PnP 一致的光学系，不能未经变换写成 camera_link 或 depth frame。[FAST-Calib 求解](https://github.com/hku-mars/FAST-Calib/blob/1018ecfdf9deda51b91a8a11bd11972a0b159008/src/main.cpp#L70)、[FAST-Calib2 求解](https://github.com/xuankuzcr/FAST-Calib2/blob/5cacbc2291eb063e4dfe972cb36df9a4bc2cbf6a/src/main.cpp#L130)、[视觉中心生成](https://github.com/xuankuzcr/FAST-Calib2/blob/5cacbc2291eb063e4dfe972cb36df9a4bc2cbf6a/src/qr_detect.hpp)。

唯一必要备选是原版 FAST-Calib：已有圆孔板且其中心提取在真实数据上通过时可用。它同样提供至少三场景联合标定，但仍是 ROS 1，并不能消除离线格式转换成本。无需因为检索到非官方 Humble fork 就改变首选。[原版 README](https://github.com/hku-mars/FAST-Calib/blob/1018ecfdf9deda51b91a8a11bd11972a0b159008/README.md)。

## 固定相机、末端板的 AX=YB 适配

记 `a_T_b` 把 b 坐标映射到 a。每个静止姿态 i：

- `A_i = camera_T_target_i`：由图像 PnP 输出。
- `B_i = base_T_gripper_i`：九轴 FK 输出，必须含 J1 直线轴；base_link 是固定 Y-up 环境系。
- `X = target_T_gripper`：未知但固定的板安装变换。
- `Y = camera_T_base`：未知但固定的外置相机变换。

闭环为 `A_i X = Y B_i`。它来自物理关系 `camera_T_target = camera_T_base · base_T_gripper · gripper_T_target`，故无需自己实现 AX=YB 求解器。

OpenCV 默认文档场景是末端相机+固定板，参数名对应那种物理场景，但求解器实际解同一个矩阵方程。这里作**语义重绑定**：

| OpenCV 参数/返回 | 本项目传入/解释 |
|---|---|
| `R_world2cam, t_world2cam` | `A_i = camera_T_target_i` |
| `R_base2gripper, t_base2gripper` | `B_i = base_T_gripper_i`，原样 FK，**此绑定不取逆** |
| `R_base2world, t_base2world` | 返回 `X = target_T_gripper` |
| `R_gripper2cam, t_gripper2cam` | 返回 `Y = camera_T_base` |

这不是声称 OpenCV 的 `base2gripper` 名称就是 FK 方向，而是用 API 承载 A/B 数学槽位。建议 adapter 仅暴露上表物理变量名，并用 3×3 float64 R、3×1 float64 t，避免方向和数组形状混淆。Shah 为主解，Li 可作同数据一致性诊断，不把两方法一致当作准确性证明。[4.5.4 官方 API](https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html)、[4.5.4 求解源码](https://github.com/opencv/opencv/blob/4.5.4/modules/calib3d/src/calibration_handeye.cpp#L749)。

本地 SSOT 所需为 `base_T_camera = inverse(Y)`；组合 `base_T_lidar = inverse(Y) · camera_T_lidar`。PnP 的 target 点尺寸和 FK 平移必须统一米，J1 数据不能把 mm 当 m。base_link Y-up 不应额外旋转成通用 Z-up；只有实际输入点云 frame 与求解 optical frame 不一致时才显式串接厂商/标定的帧变换。

## 实施前验证与未决

本研究做了网络核验和定点源码检查，未构建 FAST-Calib、未处理上游样例、未采集实物、未测出新外参。主会话另以本机 OpenCV 4.5.4、随机 seed 25、24 个无噪声姿态验证上述方向：Shah/Li 的 SSOT 最大元素误差分别为 9.85e-16/2.00e-15，闭环最大元素误差为 1.83e-15/4.88e-15。脚本与原始结果在 `.scratch/oscbf-reuse-wayfinder/25/check_ax_yb.py`、`ax-yb-result.json`（本地临时证据，非可移植链接）。这是矩阵槽位方向检验，不能证明真实系统 20 mm 验收合格。

1. 核验 Gemini 实际图像 optical frame、分辨率、畸变和内参来源；核验 Mid360S 铭牌、点云字段类型与强度。RGB 标定不能自动充当 depth frame 标定。
2. 锁定离线工具环境及数据转换方法，先跑上游样例；转换前后核对点数、XYZ/强度和坐标单位。尚未选定/实测具体容器镜像。
3. 收集有明显不同旋转轴和位置的末端静止姿态；只有直线轴变化或同轴转动可能退化。求解所需最少输入数量不等于足够的可观性；留出未参与拟合的姿态做闭环与投影检验。
4. 分别报告中心匹配/重投影残差、留出姿态闭环、已知物体尺寸和位置误差。训练残差不可替代外参误差估计，更不能从“1 秒标定”宣传推出安全精度。
5. 按 ADR 0003/0004 将通过验收的结果、求解器版本、板尺寸、采集集身份及 provenance 写入 `config/sensor_extrinsics.yaml`；正式记录需完整 calibrated/serial/id/time/operator/method/residual/error_estimate/from/to。未验证结果保持未标定状态。部署后校对 ament share resolved 路径、calibration_id 与内容 hash，不能仅修改源码树后宣布 runtime 生效。

以上动作是实施验收前置条件；本票可决定复用边界，不应把设备未到场或尚无标定数据改写为算法已通过验收。

本次不选定或自动安装 ROS 1 隔离环境；该方案仅是离线复用候选，需样例/设备数据验证后锁版。GPL v2 是版本来源与复用边界记录，不是禁止使用或不能复用的结论。
