# RViz 选区点云裁剪实验

日期：2026-09-27。依据：[机械臂附近的 LiDAR 局部碰撞场景研究](robot-centered-lidar-local-scene-20260927.md)。用户确认当前没有实测外参和真实九轴关节状态，使用 RViz 选区实验。

## 操作

在 `livox_frame` 中点击机械臂中部的一个点，将其作为三维区域中心。保留区域采用与该坐标系轴向一致的长方体，`roi_size` 为完整的 X、Y、Z 尺寸，单位为米。默认 `[2.0, 2.0, 2.0]` 是显示参数，尚未依据制动距离或全臂范围验证。

已运行的 `/livox/lidar` 来自 MID-360S。实验节点只订阅点云，不配置雷达或发送机械臂指令。源码启动方式：

```bash
cd /home/lsn/robot/robot_safecontrol
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p .scratch/lidar-crop-experiment/runtime .scratch/lidar-crop-experiment/ros-log
export ROS_DOMAIN_ID=95
export ROS_LOG_DIR="$PWD/.scratch/lidar-crop-experiment/ros-log"
export TMPDIR="$PWD/.scratch/lidar-crop-experiment/runtime"
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
python3 -m robot_safecontrol_moveit.lidar_crop_experiment
```

另一个终端加载相同 ROS 环境和 `ROS_DOMAIN_ID=95`，打开实验配置：

```bash
rviz2 -d /home/lsn/robot/robot_safecontrol/config/lidar_crop_experiment.rviz
```

1. 工具栏选择 **Publish Point**，点击机械臂区域中部的可见点。
2. **ROI Preview** 自动显示区域内点云；**ROI Boundary** 显示绿色边框。
3. 确认全部连杆、工具及希望观察的附近环境处于边框内部。边框轴向属于 `livox_frame`，安装倾斜时可能与地面方向不同。
4. 需要查看原始全景时勾选 **Raw Reference**。这个图层只是显示参考，不改变裁剪结果。

RViz `Publish Point` 使用当前 Fixed Frame 发布选点；实验配置将其设为 `livox_frame`。对应接口已核对 [Humble PointTool 源码](https://raw.githubusercontent.com/ros2/rviz/humble/rviz_default_plugins/src/rviz_default_plugins/tools/point/point_tool.cpp) 和 [MarkerDisplay 源码](https://raw.githubusercontent.com/ros2/rviz/humble/rviz_default_plugins/src/rviz_default_plugins/displays/marker/marker_display.cpp)。

调整区域尺寸、恢复全景选区及保存参数：

```bash
ros2 param set /lidar_crop_experiment roi_size '[3.0, 2.0, 2.0]'
ros2 param set /lidar_crop_experiment enabled false
ros2 param dump /lidar_crop_experiment > .scratch/lidar-crop-experiment/selected-region.yaml
```

重新使用保存的参数：

```bash
python3 -m robot_safecontrol_moveit.lidar_crop_experiment --ros-args \
  --params-file .scratch/lidar-crop-experiment/selected-region.yaml
```

包安装后提供 `ros2 run robot_safecontrol_moveit lidar_crop_experiment`，RViz 配置安装到 package share 的 `config/lidar_crop_experiment.rviz`。

## 数据与实现

| 接口 | 行为 |
| --- | --- |
| `/livox/lidar` | 原始输入保留；实验默认输入，可通过只读启动参数 `input_topic` 修改 |
| `/lidar_crop/center` | RViz 发布的 `PointStamped`；只接受与启动参数 `frame_id` 相同的坐标系 |
| `/lidar_crop/points` | 选区启用后发布区域内全部有效点；等待选区时不发布 |
| `/lidar_crop/preview` | 等待选区时显示原始输入，选区启用后显示当前帧裁剪结果；输入中断时清除预览 |
| `/lidar_crop/region` | 三维边框，停止收到输入后 0.5 秒自动失效 |
| `/lidar_crop/status` | 每 0.1 秒发布状态、最近输入年龄、最近一帧的计数和参数 |

状态为 `WAITING_SELECTION`、`CROPPING` 或 `NO_RECENT_INPUT`。`last_frame` 内的参数与计数来自同一帧。输入年龄使用本机 monotonic 时钟，仅表示接收中断时间，不代表传感器测量年龄。超过 0.5 秒没有新的输入时，节点向 `preview` 发布保留原 header 与 fields 的零点消息以清除显示，并删除区域边框；`points` 不发布用于清屏的消息。状态明确报告 `NO_RECENT_INPUT`，恢复输入后继续发布新帧。定时检查周期为 0.1 秒；这个显示超时参数尚未作为碰撞控制延迟上限验证。

通用配置和现场观察配置均设置 `Decay Time: 0`，每个新帧替换上一帧，不累积历史点。RViz 中 `0` 的含义已经核对 [Humble PointCloudCommon 源码](https://raw.githubusercontent.com/ros2/rviz/humble/rviz_default_plugins/src/rviz_default_plugins/displays/pointcloud/point_cloud_common.cpp)；带 XYZ 字段的空消息可进入显示更新，见 [PointCloud2Display 源码](https://raw.githubusercontent.com/ros2/rviz/humble/rviz_default_plugins/src/rviz_default_plugins/displays/pointcloud/point_cloud2_display.cpp)。原始参考图层默认关闭。

[point_cloud_crop.py](../../../../src/robot_safecontrol_moveit/point_cloud_crop.py) 通过 ROS `sensor_msgs_py` 读取坐标，通过 NumPy 执行含边界的 AABB 筛选。去除非有限坐标和零向量，区域内其余点全部保留。结果按完整点记录复制，保留 `intensity`、`tag`、`line`、`timestamp`、字段布局、frame 和采样时间；不执行随机限点、体素降采样或自体删除。输入数据保持不变。当前支持无行间填充的 little-endian PointCloud2，不支持的布局会明确报错。

[lidar_crop_experiment.py](../../../../src/robot_safecontrol_moveit/lidar_crop_experiment.py) 持有所有 ROS I/O。默认互斥 callback group 串行处理选区和点云，点云沿用 `state_stream_qos()` 的 `BEST_EFFORT`、`VOLATILE`，将该节点的点云队列 depth 设置为 1；RViz 点云订阅 depth 与 `Queue Size` 同样为 1。选点和 marker 使用可靠传输。输入与两个点云输出经过 ROS 名称解析后必须互不相同。输入 frame 不匹配或结构不支持时进程报告错误并退出；原始采集进程继续独立运行。没有碰撞控制消费者订阅这些实验输出。

## 已执行的验证

工作目录基于 commit `5530ab8e345d02ef1d0943c7fd90b6436f7655d0`，包含本次未提交修改。环境为 ROS 2 Humble、Python 3.10、Linux 6.8.0-138-generic。

算法与 ROS 测试覆盖边界点、零向量、非有限坐标、空区域、字段逐字节保留、输入不变、区域中心平移、无效配置、布局拒绝、真实 ROS 选点传输、尺寸修改、恢复全景、输入过期、预览清除与 topic 冲突。恢复输入时使用位置不同的点云，验证输出仅包含新帧记录，且原始采样时间保持不变。

本次连同 Livox 消息生成回归运行了 22 项测试，全部通过，退出码为 0。

wheel 构建和仓库内独立安装均成功。安装副本的模块导入、ROS node 初始化、console entry 及 RViz 资源一致性检查通过，退出码为 0；构建与安装文件位于 `.scratch/lidar-crop-experiment/`。`git diff --check` 与新增 Python 文件的语法检查通过。

实际 RViz node `/rviz` 已确认订阅 `/lidar_crop/preview`、`/lidar_crop/region`，并提供 `/lidar_crop/center` publisher。用户已在 RViz 选择并确认近似机械臂区域，运行状态为 `CROPPING`。

真实数据验证使用独立逐轴筛选生成预期点索引，并逐字节比较完整记录。测试区域为 `livox_frame` 中心 `[2, 0, 0]`、尺寸 `[2, 2, 2]` 米。这组坐标用于可重复的数据处理验证，未标注为机械臂位置。实时测试使用独立的 `/crop_validation` namespace，不改变用户在 `/lidar_crop_experiment` 中的选区。

| 数据来源 | 验证帧数 | 输入点数 | 保留点数 | 保留比例 | 裁剪函数耗时 P50 / P95 / 最大值 |
| --- | ---: | ---: | ---: | ---: | --- |
| 真实录制 rosbag | 52 | 1,043,328 | 112,857 | 10.82% | 1.47 / 1.79 / 2.02 ms |
| 实时订阅，10.04 秒 | 100 | 2,006,400 | 217,194 | 10.83% | 1.47 / 1.56 / 1.77 ms |

以上两个命令退出码均为 0，所有匹配帧的记录、header、fields 和点数均通过检查。耗时只测量裁剪函数，包含坐标读取、筛选和输出消息构造，不代表从目标出现到控制响应的总延迟。表中保留比例包含无效零向量的去除。

录制文件位于 `.scratch/t1-hardware-session-20260927/connectivity-sample/`，其 `connectivity-sample_0.db3` SHA-256 为 `999bd76278e39a32ed053823f0be48cfea50ae111b5a1109d1b3624f99e371ab`。JSON 证据保存在 `.scratch/lidar-crop-experiment/bag-validation.json` 与 `live-validation.json`。

复现命令使用前述环境，测试 ROS 域为 96，实时实验域为 95：

```bash
python3 -m pytest tests/test_point_cloud_crop.py tests/test_lidar_crop_experiment.py \
  tests/test_livox_mid360_ros_node.py \
  -q --basetemp="$PWD/.scratch/lidar-crop-experiment/pytest"
python3 scripts/validate_lidar_crop.py \
  --bag .scratch/t1-hardware-session-20260927/connectivity-sample \
  --center 2 0 0 --size 2 2 2 \
  --output .scratch/lidar-crop-experiment/bag-validation.json
python3 scripts/validate_lidar_crop.py --live --seconds 10 \
  --center 2 0 0 --size 2 2 2 \
  --output .scratch/lidar-crop-experiment/live-validation.json
```

## 用户选定的机械臂区域

用户通过 RViz 点击选区，并结合现场照片确认绿色框内大致为机械臂所在范围。读取到的 `livox_frame` 坐标如下，单位为米：

- `roi_center = [0.1058053970336914, -1.2634875774383545, 0.4365668296813965]`
- `roi_size = [2.0, 2.0, 2.0]`
- `enabled = true`

参数保存在 `.scratch/lidar-crop-experiment/robot-region.yaml`，可以通过前述 `--params-file` 方式恢复。这些坐标来自 RViz 的三维选点，未从照片估算外参或机械臂姿态。

使用该区域在 `/crop_validation` namespace 进行 10.04 秒实时验证，100 帧、2,006,400 个输入点共保留 179,185 个点，保留比例 8.93%，每帧保留 1,613–2,011 个点。完整点记录逐字节检查通过，命令退出码为 0。裁剪函数耗时 P50 为 1.52 ms，P95 为 1.72 ms，最大值为 2.09 ms。JSON 证据位于 `.scratch/lidar-crop-experiment/robot-region-validation.json`。

用户截图同时启用了 `Raw Reference`，灰色全景用于参照；取消勾选后仅显示绿色框内的彩色点云。现场照片显示机械臂沿工作台和导轨展开。该人工选区确认的是主体的大致位置，全部连杆、末端、导轨行程及附近障碍物的覆盖仍需根据三维数据与允许运动范围检查。

实时观察配置保存在 `.scratch/lidar-crop-experiment/robot-region.rviz`：显示 `/lidar_crop/preview` 的最新一帧，点大小为 2 pixels，观察距离为 4 米，观察中心使用已保存的选区中心，关闭网格和灰色全景。输入中断时预览清除，状态表示没有新数据。

使用该配置重启实验节点与 RViz 后，连续 5 秒按原始采样时间匹配并验证 50 帧实时数据。每帧裁剪结果均与对应原始帧的独立筛选结果逐字节一致，预览与当前裁剪结果一致，每帧保留 1,477–1,815 个点。原始点记录的时间戳显示，单帧采样跨度 P50 为 99.09 ms，最大值为 100.10 ms。验证程序退出码为 0，证据位于 `.scratch/lidar-crop-experiment/latest-frame-validation.json`。

```bash
rviz2 -d /home/lsn/robot/robot_safecontrol/.scratch/lidar-crop-experiment/robot-region.rviz
```

## 适用范围

当前结果验证人工选区的显示与数据处理。区域固定在传感器坐标系中，机械臂运动后不会跟随；现场最小目标保留率和允许运动范围内的覆盖尚未验证。

逐帧显示消除多帧叠加产生的历史轮廓。当前约 10 Hz 的一帧仍包含一个扫描时间段内的采样，运动中的物体可能产生帧内变形；本实验没有进行运动补偿。不能将显示无重影直接解释为实时碰撞检测已经完成，也不能用插值点填补未观测区域作为碰撞证据。

机械臂自身点和附近环境点一起保留。完整碰撞检测仍需要实测外参、真实关节反馈、全臂与工具几何、自体识别、运动期间覆盖以及占据状态的时效判定。区域内无点、选区变化或输入中断均不提供空闲证据。原始点云继续保留，后续射线处理可以使用远处回波穿过局部区域的观测信息。
