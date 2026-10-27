# 激光跟踪仪零位标定准备

## Destination

形成覆盖现场采集、离线求解和独立验收的标定准备方案，使后续开发与现场测量具备清楚的输入条件和完成标准。

## Notes

- 2026-10-26，用户确认上述目标，说明设备、靶球及安装座、测量数据均未具备。
- 用户此前描述的测量位置为机械臂末端。引用会话中的图片未通过当前读取工具返回，具体安装几何待确认。
- 本地工作由 [Trellis 任务](../../../.trellis/tasks/10-26-laser-tracker-calibration-preparation/prd.md)管理，采用 wayfinder、grilling、domain-modeling 和 research 方法。
- 正式项目归属沿用完整项目地图中的模型接口与现场标定任务；本目录保存该专题的本地准备资料与决定草稿。
- 后续会话从[决定与依赖](decisions.md)选择已有前提满足的事项；研究结论和人的决定各自保留来源。

## Decisions so far

- [现有项目怎样提供标定所需的关节与模型数据](tickets/project-interfaces.md)：已核查关节换算、名义模型、J9 几何及真实反馈接口条件。
- [单靶球测量能够确定哪些零位参数](tickets/measurement-observability.md)：已明确基座与 J1、安装坐标与 J9 的等效关系及独立基准要求。

## Not yet specified

- 真实样本显示的误差规律可能要求补充几何、回差或温度相关模型；具体需要增加哪些参数由测量证据决定。

## Out of scope

- 本轮之外的交付：设备采购、正式现场采集、生产标定值写入、电机零点改写、控制功能开发与实机验收。

## 资料

- [决定与依赖](decisions.md)
- [现场准备与数据要求](field-preparation.md)
- [离线求解与验收设计](offline-design.md)
- [测量可辨识性研究问题](tickets/measurement-observability.md)
- [项目接口与模型研究问题](tickets/project-interfaces.md)
- [本轮验证记录](../../../.trellis/tasks/10-26-laser-tracker-calibration-preparation/research/preparation-validation.md)
