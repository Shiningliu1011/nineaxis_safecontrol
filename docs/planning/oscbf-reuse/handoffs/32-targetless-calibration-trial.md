# 无标定板雷达—相机标定：隔离可行性试验报告

- 日期：2026-09-09 起；本版 2026-09-10 更新，加入了真实摆放上的完整求解与简并分析。范围：T5 的雷达—相机无板外参路线可行性试验，不是正式标定交付。
- 关联：研究[targetless-lidar-camera-calibration.md](../research/targetless-lidar-camera-calibration.md)、T5 交接[25-calibration-toolchain.md](25-calibration-toolchain.md)、驱动交接[31-official-sensor-drivers.md](31-official-sensor-drivers.md)。
- 结论：**有条件可行**。工具链、本机数据流、官方 `calibrate` 端到端都已实测跑通，并在真实摆放上求出了一组外参（`preprocessed-new2/calib.json`）；但这组场景的 NID 代价面存在**周期性简并**——解可以沿天花板格栅周期整体平移一格而代价更低，俯仰角基本不受约束，官方结果在工具自己的粗代价里甚至不是局部极小。因此这组数值**不可作为外参使用**。限制因素是场景（共同视场几乎只有 3 m 外的周期井格天花板），不是工具；需要固定件保证共同视场内有非周期近场结构后重采。
- 证据目录：`.scratch/targetless-calibration-trial/`（原始 bag、实时采集、预处理产物、每次搜索的 JSON、日志、截图）。

## 已实测通过的部分

### 1. 工具链（官方 Humble 镜像）

- `docker pull koide3/direct_visual_lidar_calibration:humble` 成功；本地镜像 digest `sha256:f7363cc3deadc2419f3527a7b9ec7bdabaa3031fd8e1b9c6cc46666ae1301bbb`，5.39 GB，amd64，构建于 2026-01-02。
- 上游源码 commit `02a0dc039f5509708f384be4ff3228e0ae09352d`（2026-01-02）克隆到 `upstream/` 供核对；`preprocess --help` 与官方文档一致。
- 四个程序 `preprocess / initial_guess_manual / initial_guess_auto / calibrate / viewer` 均在镜像内可执行。
- 输出方向已按源码核对：`calibrate` 优化的是 `T_camera_lidar`，写入 `calib.json` 的 `results.T_lidar_camera` 是它的逆，即 `p_lidar = T_lidar_camera * p_camera`；与本项目若需“雷达到相机”应取逆的提醒一致。
- 官方文档示例所需样例数据集托管在 Zenodo；zenodo.org 持续返回 504/超时，**官方样例未能下载**，因此工具端验证改用本机真实数据完成。

### 2. 本机传感器数据流

驱动沿用 31 号交接的隔离前缀，未改产品代码：

- 雷达：`livox_ros_driver2_node`，`xfer_format=0`，`MID360s_online.json`（主机 192.168.1.5，雷达 192.168.1.115），`/livox/lidar` 实测 10.0 Hz，PointCloud2，`livox_frame`，point_step 26，字段 x/y/z/intensity(float32)+tag/line(uint8)+timestamp(float64)。
- 相机：`gemini_330_series.launch.py`（`depth_registration:=true enable_colored_point_cloud:=true`），`/camera/color/image_raw` 与 `/camera/color/camera_info` 实测约 23–30 Hz；CameraInfo `distortion_model: plumb_bob`，`d` 8 个系数，`k = [366.465, 366.479, 319.416, 237.717]`，frame `camera_color_optical_frame`。
- 采集 bag：`bags/trial03`，21.4 s，雷达 214 帧、彩色图 641 帧、CameraInfo 642 帧（用户在会话中途调整了相机角度，此前 trial01/trial02 已作废并归档到 `bags-archive/`）。trial03 之后又调整过一次角度，因此后续分析全部改用实时采集。
- 实时采集：`data/scene-new2/`，不经 bag 直接抓 15 帧雷达（10 Hz，1.5 s）+ 1 帧彩色图，共 300,000 点（`capture.json`）。**其中 113,293 点是 Livox 无回波的零值点（r=0），分析前必须剔除**；剔除后 186,707 点。本次求外参用的就是这份数据。

预处理（官方容器，显式指定话题，不用 `-a` 自动识别）：

```bash
docker run --rm -v "$PWD/bags:/tmp/input_bags" -v "$PWD/preprocessed-trial03:/tmp/preprocessed" \
  koide3/direct_visual_lidar_calibration:humble \
  ros2 run direct_visual_lidar_calibration preprocess \
  --image_topic /camera/color/image_raw --points_topic /livox/lidar \
  --camera_info_topic /camera/color/camera_info -i intensity /tmp/input_bags /tmp/preprocessed
```

trial03 的结果：2,418,295 点累积点云、相机图 `trial03.png`、LiDAR 强度全景（1920×960 等距柱状）、索引图、`calib.json`。预处理对图像与强度都做了直方图均衡，LiDAR FoV 实测 175.2°（走 equirectangular 分支）；日志提示 Livox 逐点时间戳为纳秒量级并被自动换算。

### 3. 工具端到端自测（合成真值，通过）

用已知 `T_camera_lidar`（相机在雷达 +x 方向 1 m、y +0.2 m、z −0.1 m，俯仰 10°）把同一份累积点云渲染成“相机图”替换进预处理目录，再把初始值扰动 **0.811°/0.113 m** 后交给官方 `calibrate --auto_quit --background`。优化器从 NID 0.892 收敛到 0.815，恢复出的位姿与真值相差 **0.197°/0.0057 m**。即官方 `preprocess → 初始值 → calibrate → calib.json` 全链路在本机数据格式上可用（证据：`results/synth_dataset_gt.json`、`preprocessed-synth/calib.json`、`data/synth_compare.png`）。

> 复盘说明：最初解读该结果时曾把四元数矢量部当成旋转向量（`cv2.Rodrigues` 误用）而误判为 66° 偏差，属分析脚本 bug，与工具无关；已修正。

## 真实数据端到端：初始值 + calibrate 已跑通（2026-09-10 新增）

**数据集**：实时采集不能直接喂官方 `preprocess`（它吃 bag），改用 `scripts/make_official_dataset.py` 逐条复刻 `Preprocess::get_image_and_points`（rgb8→mono8 + 直方图均衡；剔除 `|p|<1.0 m`；2 mm 体素；按秩的强度均衡），生成官方格式数据集 `preprocessed-new2/`（142,716 点 + 640×480 灰度图 `new2.png`），`calib.json` 的 camera 块取自 trial03 的 CameraInfo。

**初始值**（不走 GUI 人工选点，改用几何法，避免编造点对）：天花板平面法线（雷达系）× 天花板格栅消失点（相机系）→ 最小旋转；再对 ψ/h/a/b 做网格搜索（ψ=244°、h=2.55 m、a=0.6、b=−0.3，自身 NID 0.92317），最后 6 自由度 Nelder-Mead 收敛到 0.92175，写入 `calib.json` 的 `results.init_T_lidar_camera`（完整记录见 `results/init_new2.json`）。

**官方 `calibrate`**：

```bash
docker run --rm -v "$PWD/preprocessed-new2:/tmp/preprocessed" \
  koide3/direct_visual_lidar_calibration:humble \
  ros2 run direct_visual_lidar_calibration calibrate --auto_quit --background /tmp/preprocessed
```

优化器从初始值移动了 **1.702° / 0.2002 m** 收敛并写回 `preprocessed-new2/calib.json`：

- `results.T_lidar_camera`：t = (−0.9936, 0.3032, −0.2816)，q = (−0.3145, −0.5534, 0.6732, −0.3763)
- 用独立复刻的 NID（`scripts/official_nid.py`，逐行对应 `cost_calculator_nid.cpp` + `view_culling.cpp`）算：初始值 0.95814 → 结果 **0.95440**（带深度缓冲剔除）/ 0.95411（不剔除）。官方终端打印的轨迹（0.9583 → 0.9543）与复刻一致到 1e-4 量级；该 stdout 当时未落盘，可复现数值以复刻为准。

**这组位姿的物理含义**：相机光心在雷达系 (−0.994, 0.303, −0.282)，距雷达 1.076 m（其中沿房间竖直向上 0.72 m、水平 0.80 m）；反过来雷达光心在相机系 (−0.704, +0.738, +0.344)，即相机下方 0.74 m、像平面之前 0.34 m，**偏离光轴 71.4°**，投影到约 (−430, 1023)——远在 640×480 图像之外。也就是说相机根本看不到雷达，两者只有“远处天花板/上部墙面”这一块共同可见区域。这与下面的简并结论一致（渲染核对：`results/render_official_new2.png`、`planes_official_new2.png`、`splat_official.png`）。

## 为什么这组数值不能用：代价面简并

### 1. 最深的谷正好是一个格栅周期

围绕结果做 ±0.6 m / ±0.1 m 的细扫（`results/nid_fine_scan_new2.json`、`nid_fine_rot_new2.json`，基准 NID 0.95440）：

| 扰动轴 | 最小值位置 | 该处 NID | 与基准差 |
|---|---|---|---|
| 相机 x | −0.07 m | 0.95359 | −0.00081 |
| 相机 y | −0.06 m | 0.95403 | −0.00037 |
| 相机 z | **+0.30 m** | **0.95171** | **−0.00268** |
| 格栅 u | +0.60 m（扫描边界） | 0.95385 | −0.00055 |
| 格栅 v | −0.06 m | 0.95398 | −0.00042 |
| rx | +0.1° | 0.95418 | −0.00022 |
| ry | +0.1° | 0.95397 | −0.00043 |
| rz | −0.1° | 0.95400 | −0.00040 |

最深的方向是相机 z 轴 +0.30 m。把相机 z 轴表示到雷达系是 (−0.007, −0.9818, 0.1896)，天花板格栅的 v 轴是 (0, 0.9981, 0.0621)，两者点积 **−0.968**——即 +0.30 m 的相机 z 位移几乎就是沿格栅 v 轴 −0.29 m，**正好一个格栅周期（0.300 m）**。天花板是周期性格栅，代价面自然对“整格平移”不敏感；这 0.0027 的改善是混叠，不是更好的解。

### 2. 俯仰角基本不受约束

ry 在 0…+2° 内基本持平（最低 0.95397，整段低于基准 0.95440），而 rx/rz 的谷都在 ±0.1° 内、再往外单调上升。即这套数据只能定住 2 个旋转自由度，第 3 个（俯仰）几乎自由。

### 3. 官方结果在工具自己的粗代价里不是局部极小

官方粗扫（`results/official_nid_scan_new2.json`）：沿 tz 一路下降到 +0.5 m（0.95277，仍未回升），沿 rz 一路下降到 −5°（0.95393）。`calibrate` 停下来的点，在它自己的代价函数里还能继续改善——这是简并最直接的证据。

### 4. 两个独立的图像“上方向”估计互差 1.71°

- 房间竖直线的消失点（相机系）：u_c = (0.0524, −0.9911, 0.1226)，23 条内点，残差中位数 0.64°；
- 天花板格栅的消失点（相机系）：n_cam = (0.0394, −0.9946, 0.0959)；
- 两者夹角 **1.71°**；官方结果的旋转离 u_c 0.41°、离 n_cam 1.94°（初始值 1.46°/0.25°）。200 次 bootstrap：u_c 波动中位数 1.42°、n_cam 0.27°、两者夹角中位数 2.09°（`results/up_direction_new2.json`、`up_direction_bootstrap_new2.json`）。

即：仅凭图像能给出的“上”只有 1–2° 精度，而官方优化把旋转只挪了 1.70°——**旋转的改变量与图像本身的噪声量级相同**，无法判定它真的改善了。

### 5. 独立指标无法区分候选

- Canny 边缘距离（`results/edge_metric_new2.json`）：天花板 RANSAC 前三个平面上，12 个候选的差异只有几像素，且排序互相矛盾——tz+0.5 在平面 0/4 上反而最好（8.60 px / 2.74 px），在平面 1 上最差。
- 格栅交点残差（`results/junction_residual_new2.json`）：官方中位数 −9.7 px，而 ±0.3 m 的候选是 −4.1/−4.7 px（更好）。该指标整体偏置为负，本身不是可靠裁判，但显然也不能排除混叠解。

## 场景几何：简并的成因

（分析集：剔除 113,293 个 Livox 零值点后的 186,707 点）

- 雷达安装倾角 **60.8°**（天花板法线 z 分量 0.4875），视场仰角 −5.5°…+53.8°；92.0% 的点在 0° 以上，77.4% 在 +10° 以上，64.3% 落在雷达上方 3.0–3.6 m 的天花板带。
- 方位角在 +x 附近有“死楔形”：|az|<45° 只有 8.5%（均匀应为 25%）。
- 天花板是**井格（coffer）结构**：周期 0.300 m、格栅方向 27.5°，相对主平面还有约 +0.01 m（面板）、−0.23 m（梁）、−0.16 m（梁边带）三层（`data/scene-new2/ceiling_grid.json`、`ceiling_plane.json`）。
- 近场主平面（桌面，r<2.5 m 的 RANSAC）法线与天花板法线差 2–3°，雷达光心在该平面之上约 4 cm；剔除天花板后剩下的平面（墙）法线与天花板法线夹角 89.6–90.0°（墙面竖直）。三处独立结构都指向同一个“上”，说明雷达自身姿态自洽——问题不在姿态估计，而在共同视场里**只有周期性结构**。
- 本次采集**没有 IMU 数据**：驱动日志显示设备端已启用 IMU（`successfully enable Livox Lidar imu`），但 `bags/trial03/metadata.yaml` 只有 3 个话题（图像/相机信息/雷达），IMU 话题没有被录制。姿态无法用 IMU 交叉验证，只能靠上面的平面一致性。

## 摆放建议（固定件与整机都适用）

依据：本次失败的摆放 + 两个传感器的实测视场（雷达竖直视场 −5.5°…+53.8°，相机约 82°×66°，即 ±41°/±33°）。

| 项目 | 本次失败摆放 | 建议 |
|---|---|---|
| 雷达姿态 | 前倾 60.8°，视场大部分打在周期天花板上 | 保持原装竖直姿态；需要看低处结构时与相机**一起**下俯 ≤20° |
| 相机光轴 | 俯视桌面，与雷达不同向 | 与雷达同向：水平，或与雷达同角度下俯 |
| 基线 | 1.08 m | 10–30 cm（不超过约 0.5 m） |
| 共同视场结构距离 | 中位 9.6 m（`data/scene-new2/overlap_sweep.json`） | 1–5 m |
| 共同视场内容 | 周期 0.300 m 的井格天花板 | 墙角、家具、门框等**非周期**结构，至少 2–3 种深度/朝向 |
| 机械 | 角度在会话中调整过两次 | 同一块刚性板、两处螺栓固定，USB 线做应力释放；采集期间绝对不动 |

**视场带怎么算**：
- 雷达竖直 + 相机水平：共用带 **−5.5°…+33°**，3 m 处约 0.9–3.1 m 高——适合墙面、门框、柜子。
- 目标结构低于传感器高度（桌子、椅子、箱子）时：两者**一起**下俯 15–20°，共用带变为约 −25°…+13°，把家具纳入共同视场。
- 关键是两个传感器同向、同俯仰；只转一个正是这次的问题。

**避开**：单一平面（一面白墙）、周期结构（井格天花板、百叶、地砖）、8 m 以上的远处走廊、屏幕/窗户（纹理变化或过曝）。

**装好后的自检**（正式标定前就能做）：
1. 用 `scripts/overlap_sweep.py` 一类的检查确认共同可见方向的中位距离落在 1–5 m（本次是 9.6 m）；
2. 采一组 10–15 s 静止数据，跑完 preprocess → 初始值 → `calibrate`；
3. 检查 NID 谷在 6 个自由度上是否都单调回升，尤其看有没有 0.3 m 周期的第二个谷（本次的坑）；
4. 把整套装置挪到第二个视角再采一组，两次求解之差应远小于验收线——重复求解一致性是目前唯一能主动发现简并的检验。

## 图形界面已验证可用

官方 `initial_guess_manual` 需要 OpenGL 三维视图。容器内没有 `/dev/dri`（未装 nvidia-container-toolkit），但 Mesa llvmpipe 软件渲染可用；关键是必须挂载 X socket 与正确的 Xauthority：

```bash
docker run --rm --net host \
  -e DISPLAY=$DISPLAY -e XAUTHORITY=/root/.Xauthority \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e GALLIUM_DRIVER=llvmpipe \
  -v /run/user/1000/gdm/Xauthority:/root/.Xauthority:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$PWD/preprocessed-new2:/tmp/preprocessed" \
  koide3/direct_visual_lidar_calibration:humble \
  ros2 run direct_visual_lidar_calibration initial_guess_manual /tmp/preprocessed
```

实测窗口正常出现（三维点云视图 + 图像窗口 + control 面板，含 0/90/180/270° 图像旋转、T_lidar_camera 手柄、Add picked points / Estimate / Save），见 `data/gui_shot6.png`。注意容器内窗口可能被图像窗口遮挡，需挪开；`--net host` 与 `/tmp/.X11-unix` 二者缺一不可。

人工选点步骤（官方定义，本次未执行——本次初始值来自上面的几何法，勿视为已人工选点）：

1. 在三维视图里右键点一个场景结构点（右侧点击=拾取 3D）。
2. 在图像窗口里右键点同一结构（右侧点击=拾取 2D）；图像窗口可用 0/90/180/270° 按钮旋转以方便对应。
3. 点 “Add picked points”；重复至少 3 组，建议 6 组以上，覆盖不同深度与方向。
4. 点 “Estimate” 得到初始位姿（RANSAC + 鲁棒最小二乘），再用 “Save” 写入 `calib.json` 的 `results.init_T_lidar_camera`。
5. 之后 `calibrate --auto_quit --background` 做 NID 精配准，`viewer` 查看投影与对齐。

## 附：上一版摆放（trial03）上失败的 9 条路径

这些是在角度调整前的旧摆放上试的，留档避免重复劳动：

| 路径 | 结果 | 证据 |
|---|---|---|
| 旋转网格搜索（假设相机在雷达原点） | NID 全在 0.96 附近，无相关 | `results/nid_coarse_trial03.json` |
| 天花板平面约束 + 3 自由度 NID 网格 | 最优 0.929，投影仅覆盖图像一小块 | `results/nid_plane_trial03.json` |
| 300 次多起点 Nelder-Mead | 最优 0.881，仍接近“不相关” | `results/nid_multistart_trial03.json` |
| Open3D FPFH+RANSAC 全局配准（全场景 / 中高度带） | fitness=0，无对应 | `results/o3d_registration.json`、`o3d_reg_mid.json` |
| 房间平面匹配（天花板/墙） | 深度云未提取出竖直墙面，无一致匹配 | `results/plane_align_trial03.json` |
| SIFT 特征 + PnP（相机图 vs 全景） | 仅 15 组匹配，PnP 失败 | `results/feature_pnp_trial03.json` |
| SIFT 逐 yaw 匹配 + PnP | 最佳 yaw=160° 98 组匹配，但 PnP 未通过 | `results/sift_yaw_trial03.json` |
| 天花板 2D 相位相关（旋转+平移） | 峰值响应仅 0.084，视为噪声 | `results/ceiling/` |
| 细网格（2°/0.1 m，±2 m）平面约束搜索 | 最优 0.923，且最优落在搜索边界；边界外投影点数骤减，NID 因“点少”而虚低，属退化而非真解 | `results/nid_plane_fine_trial03.json` |

机制诊断（合成真值试验 `results/synth_gt.json`）：把点云按已知变换渲染成“相机图”后，正确位姿的 NID 为 0.56，而 5°/0.5 m 网格搜索最优只有 0.95——NID 极小值比粗网格窄得多，这正是官方要求人工给初始值的原因。端到端自测还给出量级参考：扰动 0.8°/0.11 m 时 `calibrate` 能收敛，即真实数据需要的初始值精度在**约 1°/0.1 m 量级**，比 5°/0.5 m 的搜索网格细一个数量级。

## 建议的下一步

1. **按上面「摆放建议」重装**：两个传感器同向、基线 10–30 cm、共同视场里放 1–5 m 的非周期近场结构（墙角/家具/门框）。当前摆放“相机看桌面、雷达看天花板”，共同可见的几乎只有 3 m 外的井格天花板——这是本次简并的直接原因。
2. 条件允许时从 2–3 个不同位置/视角各采一组，用同一组外参分别求解并互相对比。重复求解一致性是目前唯一能主动发现简并的验证手段。
3. 固定件到位后重采 → 重跑 `preprocess`（或 `make_official_dataset.py`）→ GUI 人工选点（或本文的几何法）给初始值 → `calibrate` → 用 `viewer` 与独立指标（边缘距离/交点残差/留出场景）验收。
4. 本次试验的临时外参一律不写入 SSOT、不设 `calibrated=true`，T5 保持 OPEN。

## 未决限制

- 本次求出的外参因代价面简并**不可用**；没有重复求解一致性、残差统计或留出场景验证。
- 官方 `calibrate` 的 stdout 未落盘（终端观察到 0.9583→0.9543）；可复现数值来自 `scripts/official_nid.py` 复刻。
- 官方样例数据集因 Zenodo 不可达未下载，工具端验证只用本机数据完成。
- trial01/trial02 因相机角度调整作废；trial03（bag）之后又调整过一次角度，最终分析用的是 `scene-new2` 实时采集。
- 相机内参直接采用设备自报 CameraInfo（plumb_bob，8 系数），未做独立内参标定。
- 两传感器硬件时间同步未做；本次采集不构成时间同步证据。
