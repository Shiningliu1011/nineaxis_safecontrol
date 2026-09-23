## 真机部署

**真机执行端**:
目标职责是订阅命令流，经安全网关和硬件传输发送真实执行命令，并将真实反馈转换成状态流。当前 `hardware_bridge` 仅提供 fail-closed containment：sim 不创建硬件控制 I/O；shadow 只记录请求/拒绝，不收发 CAN、不发布真实硬件状态；live 禁用。真实 SocketCAN、反馈 freshness/watchdog 和真机准入见 [GitHub #13](https://github.com/Shiningliu1011/nineaxis_safecontrol/issues/13)。
_Avoid_: hardware bridge、CAN bridge

**安全网关**:
命令流的唯一卡口——超时/故障/限幅违例时输出零速保持命令并锁存停车原因，直到外部健康确认后人工恢复；网关只检验不重塑。
_Avoid_: safety gate、command validator

**解析障碍物**:
点云经聚类后拟合的球/圆柱几何障碍物，以包络球进入控制内核的 obs_* 接口；与稠密点云/ESDF 不同，它是可验证的几何单元。
_Avoid_: obstacle shape、fitted obstacle

**零位标定表**:
记录每个关节的方向符号与电机零位偏移的配置文件（hardware_joint_zero.yaml），由机械零位标记法生成。
_Avoid_: zero calibration、home offset

**DrEmpower 帧**:
电机 CAN 通信的基本单元，CAN ID = (node_id << 5) | cmd_byte，位置命令 0x19、系统命令 0x08。
_Avoid_: CAN frame、motor command
