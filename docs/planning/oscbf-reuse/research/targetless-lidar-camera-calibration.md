# 无标定板的 MID-360S—Gemini 335L 外参方案

核验日期：2026-09-09。范围：T5 的雷达—相机刚体外参；不是相机—机器人基座外参，也不是时间同步标定。本次仅研究，未安装工具、未采集或求解新外参。

## 结论

优先试 **koide3/direct_visual_lidar_calibration 的手动选对应点初始化 + 自动 NID 精化**。它不需要专用标定板，支持 ROS2 和非重复扫描 LiDAR，有 Livox 实例和结果查看器；依赖较多，但官方提供 Humble Docker 路线。第二选择是 **CloudCompare 将 Gemini 深度点云与 Livox 点云做手动选点 + ICP**：图形界面直观，能复用目前两路 PointCloud2，适合先求初值及独立几何复核；它不是针对这套跨传感器组合验证过的即插即用标定器。以上是根据官方输入条件与本地接口作出的推荐，尚无 MID-360S + Gemini335L 实测精度证据。[工具仓库](https://github.com/koide3/direct_visual_lidar_calibration)、[官方 Docker](https://koide3.github.io/direct_visual_lidar_calibration/docker/)、[CloudCompare ICP](https://cloudcompare.org/doc/wiki/index.php/ICP)

## 候选比较

| 路线 | 输入与操作 | 适合程度及限制 | 许可证/平台 |
|---|---|---|---|
| direct_visual_lidar_calibration | Image、PointCloud2（有有效 intensity）、已知 CameraInfo；在 GUI 选自然场景 2D–3D 对应点，再自动优化、查看投影 | 首选；支持非重复扫描，但并非已证实 MID-360S 专项兼容。依赖共同视野和强度纹理；不需深度相机点云 | 主项目 MIT；ROS1/ROS2，官方 Humble Docker；可选 SuperGlue 有非商业限制，首轮不需要它 |
| CloudCompare | 将两路点云导出 PLY/PCD，裁共同区域，选择至少 3 对对应点做刚体对齐，再 ICP | 上手直观；跨源深度偏差、遮挡和点密度差会影响结果。建议用于初值和复核 | GUI 主程序 GPL，核心库 LGPL；离线文件工作流，无需 ROS 版本适配 |
| hku-mars/livox_camera_calib | PCD + 图像 + 内参/畸变 + 初值，抽取环境边缘；支持多场景 | 备选；有 Livox Avia + RealSense D435i 示例，理念匹配，但当前 Ubuntu22/Humble 迁移成本更高 | GPL-2.0；官方说明 Ubuntu16/18、ROS Kinetic/Melodic、catkin |
| Livox 官方 livox_camera_lidar_calibration | 板角点手工提取；官方流程含板、多个位置及旧驱动 CustomMsg | 不作为本次无板首选。可以借鉴 PnP 原理，但自然角点替代不是官方原样流程 | 老 ROS1/Ubuntu16.04 流程 |

依据：[Koide README](https://github.com/koide3/direct_visual_lidar_calibration)、[Koide 安装与许可证提示](https://koide3.github.io/direct_visual_lidar_calibration/installation/)、[CloudCompare Align](https://www.cloudcompare.org/doc/wiki/index.php/Align)、[CloudCompare 许可](https://www.cloudcompare.org/doc/wiki/index.php/License)、[HKU 官方仓库](https://github.com/hku-mars/livox_camera_calib)、[Livox 官方仓库](https://github.com/Livox-SDK/livox_camera_lidar_calibration)。2026-09-09 GitHub API 显示 Koide 仓库未归档，最近 push 为 2026-07-22；这仅是维护迹象，不代表已在本机构建通过。

## 首选方案的可执行边界

1. 将两个设备牢固固定在一起，面对有共同可见结构和纹理的静态场景。官方对 Livox 建议静止录制 10–15 秒，至少一个 bag，最好 5–10 个。不能把旋转式 LiDAR 的移动积分步骤照搬到 Livox。相机内参须预先已知。[官方采集说明](https://koide3.github.io/direct_visual_lidar_calibration/collection/)
2. 本地已实测 `/livox/lidar` 为 10 Hz PointCloud2，包含 xyz/intensity；这满足工具输入的基本形状，但仍需查看 MID-360S 积分后的强度图是否有可匹配信息。还要录制 Gemini **彩色 Image + 对应 CameraInfo**，不能只录 `/camera/depth_registered/points`。内参与实际图像分辨率、畸变模型必须一致。本地接口证据见同目录 `official-sensor-drivers.md` 及上级 handoffs 中传感器交接记录。
3. 离线执行 `preprocess → initial_guess_manual → calibrate → viewer`。手动对应点是初始化，后续仍自动优化；官方最少 3 对，实际建议选择更多、分布广且有不同深度的明确自然结构点。该建议是工程判断，不是工具精度保证。明确指定图像/CameraInfo/雷达话题与 `intensity`，避免多个点云话题导致自动选择错误。[程序接口](https://koide3.github.io/direct_visual_lidar_calibration/programs/)
4. 输出 `T_lidar_camera` 的定义是 `p_lidar = T_lidar_camera * p_camera`，即相机到雷达；若项目要雷达到相机应取逆，且相机坐标必须对应本次图像的 optical frame。不能凭变量名字直接粘到 SSOT。[输出定义](https://koide3.github.io/direct_visual_lidar_calibration/programs/)

推荐场景是有墙角、桌边、柜体等不同朝向与深度的共同结构，并有颜色/反射强度变化。避免只有一面空白墙、大面积玻璃/镜面、重复平行结构或人员运动；这是对纹理需求及几何可观测性的工程判断。没有标定板不等于没有场景条件。

## RGB-D 点云路线如何使用

Gemini 目前注册点云 frame 为 `camera_color_optical_frame`，可与 `livox_frame` 中的点云直接作 3D–3D 配准。CloudCompare 选择要移动的 Livox 为 Data、相机点云为 Reference，则输出按这个选择理解为雷达至相机；保存完整矩阵与角色记录。两路单位都须核验为米；保持固定尺度，不启用 scale adjustment，以免把深度系统误差吸收到缩放里。先裁重叠视野、去除深度无效点及遮挡边界，再手选对应点，已有合理初值后才用 ICP。[Align 角色/尺度说明](https://www.cloudcompare.org/doc/wiki/index.php/Align)、[ICP 初值与重叠要求](https://cloudcompare.org/doc/wiki/index.php/ICP)

这里存在 RGB-D 深度系统误差以及两传感器看到的表面不同等风险，低 ICP RMS 仅表明本次拟合残差小，并不能证明真实外参准确。单平面可能不能充分约束全部自由度；不同朝向平面和跨深度结构更适合验证。上述为工程推断，尚未针对当前硬件试验。

## 验收与 T5 的关系

- 留出未参加优化的场景，检查投影边界、多距离点云对齐及重复采集求解的一致性；保存原始 bag、相机内参、工具版本、矩阵方向和残差。
- 不把论文中的“像素级”结果当成本项目 **20 mm** 达标证据；2D 投影看起来重合也不是三维误差上界。
- 无板方案可解除雷达—相机这一段对专用板的等待；机器人基座外参仍需独立观测/测量，不能由两传感器互标自动获得。
- 本研究给出试验优先级，不关闭 T5，不写运行外参，不改变已有 SSOT/验收门槛。
