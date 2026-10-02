# xy_lidar

从用户提供的 TMBS 与 DrillLidarDrv 资料中整理的 LiDAR 处理方法及原始源码参考。整理日期：2026-10-02。

## 阅读入口

- [处理方法](docs/processing-methods.md)：多尺度配准、点云预处理、Livox `tag`、坐标变换、异步结果检查。
- [与 OSCBF 项目的关系](docs/oscbf-integration.md)：现有入口、数据要求与接入前的验证条件。
- [资料来源](docs/sources.md)：压缩包身份、源码目录、依赖及校验方式。
- [源码校验清单](SHA256SUMS)：12 份原始参考文件的 SHA-256。

`reference/` 保存所选文件的原始内容，供阅读和后续适配。执行源码需要原项目的依赖与配套模块；本目录交付的是方法资料与源码快照。
