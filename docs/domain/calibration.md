## 标定记录与准入

**标定记录**:
单个传感器静态外参及其 provenance 组成的一条记录，按源存放在
`config/sensor_extrinsics.yaml` 的对应分节中（ADR 0009）。
_Avoid_: 外参来源、calibration entry

**记录身份（calibration_id）**:
一条标定记录**内容**的稳定标识：标定内容改变时它改变，文件其它部分（注释、别的源、
无关配置块）改变时它不变；用于比对「被校验的记录」与「运行时实际加载的记录」。
_Avoid_: file hash、文件版本号

**文件身份（file_sha256）**:
节点实际加载的整个文件字节的摘要，证明读到的是被检查的那份文件；它不刻画单源标定
内容是否变过。
_Avoid_: 用 calibration_id 指代文件身份

**未标定调试身份**:
某源以 `calibrated: false` 的假定几何被启用时的身份。仅当部署 profile 明确逐源允许时
才允许启动，且不构成标定准入。
_Avoid_: 未标定模式、debug mode

**标定准入**:
某个源的标定记录可否作为控制器**可信障碍输入**的判据，四个条件同时成立：矩阵数学合法、
`calibrated: true`、provenance 完整、记录身份自洽（声明的 `calibration_id` 与内容算出的
一致）。部署 profile 的逐源豁免只决定「能否启动」，不改变本判据的结论。
_Avoid_: 准入检查（那是求解后对松弛额度的检查）、admission、calibration gate
