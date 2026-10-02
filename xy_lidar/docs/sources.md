# 资料来源与校验

整理日期：2026-10-02。原始资料目录为 `/home/lsn/robot/lidar`。

## 压缩包身份

| 压缩包 | ZIP comment 中的 Git 提交标识 | SHA-256 |
| --- | --- | --- |
| `tmbs-main.zip` | `65d61973cf8351906353b23442bb097219af303a` | `ec0338a5f383ff0816a93ce8a3ad6a33cfc8a9f44bbf071d7131e83685aff9e7` |
| `drilllidardrv-master.zip` | `f926db8f5897f08c8642a1cdee8008003948656e` | `dd65a089f1aeb247fde333baae81f0f4ae754ac27214557f0addf1c7d1526bf5` |

提交标识取自压缩包元数据，未连接远程仓库验证。解压时完成压缩数据完整性检查，并逐文件核对解压内容与 ZIP 中的内容一致。

## 参考文件

`reference/tmbs/` 对应原始 `tmbs-main/`，`reference/drilllidardrv/` 对应原始 `drilllidardrv-master/`；后续相对路径保持原样。参考文件内容按原始字节保存。

| 文件 | 阅读用途 |
| --- | --- |
| [lidar_registration.py](../reference/tmbs/backend/app/algorithms/lidar_registration.py) | 多尺度 ICP/GICP 与候选质量判据 |
| [calibration.py](../reference/tmbs/backend/app/utils/calibration.py) | 外参表示、矩阵校验、Euler 与矩阵转换 |
| [preprocess.py](../reference/tmbs/backend/app/algorithms/distance_detection/preprocess.py) | 点云清理、范围筛选、体素处理、统计诊断 |
| [rigid_transform.py](../reference/tmbs/backend/app/algorithms/distance_detection/rigid_transform.py) | 两段外参组合与刚体变换 |
| [parameters.py](../reference/tmbs/backend/app/algorithms/distance_detection/parameters.py) | 采集时长与处理参数定义 |
| [livox_tag_filter.py](../reference/tmbs/backend/app/drivers/livox_tag_filter.py) | `tag` 分组掩码与配置 |
| [mid360_driver.py](../reference/tmbs/backend/app/drivers/mid360_driver.py) | 原始协议、批次采集及点云字段 |
| [requirements-base.txt](../reference/tmbs/backend/requirements-base.txt) | 原系统基础依赖 |
| [requirements-native.txt](../reference/tmbs/backend/requirements-native.txt) | 原系统计算依赖 |
| [DistanceDetectionController.cs](../reference/drilllidardrv/src/Detection/DistanceDetectionController.cs) | 结果顺序与有效状态处理 |
| [TaskResultValidator.cs](../reference/drilllidardrv/src/Detection/TaskResultValidator.cs) | 任务身份、版本及结果结构检查 |
| [LidarModels.cs](../reference/drilllidardrv/src/LidarModels.cs) | C# 输入输出字段定义 |

## 运行与维护范围

此处选择的是阅读相关的源码文件。Python 源码保留原有 `app.*` 导入，其中配准还使用原项目的 PCD 读取和日志模块；C# 源码使用原项目的控制器基类、接口及宿主依赖。运行相关算法需要完整处理这些依赖。

原 TMBS 手册指定 Python 3.11；所附计算依赖包含 `numpy<2`、`scipy>=1.10`、`open3d==0.18.0` 和 `scikit-learn>=1.2,<2`。这些是来源项目的环境声明，当前资料整理没有安装或验证该环境。

后续适配时，把经过验证的功能放入项目正常模块，并保留来源说明；`reference/` 用于保持可核对的来源快照。源码原有的错误处理、默认值与注释属于参考资料，适配实现遵循目标模块的项目要求。

## 文件校验

在 `xy_lidar` 目录运行：

```bash
sha256sum -c SHA256SUMS
```

[SHA256SUMS](../SHA256SUMS)覆盖全部 12 份参考文件。校验用于确认资料完整性；算法精度、实时性和硬件行为需要对应运行证据。
