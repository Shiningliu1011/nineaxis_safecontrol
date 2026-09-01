"""真机执行端节点（hardware_bridge）——ROS 2 薄壳，副作用经注入。

角色与 oscbf_plant 同构：订阅命令流（/oscbf_command，JointState.position，
100 Hz）→ 安全网关 → 单位换算 → DrEmpower CAN 帧 → 后端发送；
反馈帧 → 解码 → 单位换算 → 发布状态流（/mujoco_joint_states）。

hardware_mode 参数：
- sim：不参与 launch（oscbf_plant 照旧）
- shadow：记录命令与状态，无发送 API（ShadowCommandRecorder）
- live：经后端真实发送

安全网关全程生效（超时/故障/限幅违例 → 零速保持 + 锁存）；
J1 传动未标定时拒绝 J1 运动命令（fail-closed）。
"""

from __future__ import annotations

import time
from typing import Optional, Sequence

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .drempower_can import (
    CMD_POSITION_ANGLE_MODE0,
    can_id,
    decode_feedback,
    encode_position,
)
from .hardware_contract import (
    ActuatorLimitProfileLike,
    CommandSafetyGate,
    HardwareState,
    PositionCommand,
    ShadowCommandRecorder,
    WatchdogConfig,
)
from .robot_spec import DEFAULT_JOINT_NAMES
from .ros_conventions import (
    JOINT_STATE_TOPIC,
    OSCBF_COMMAND_TOPIC,
    state_stream_qos,
)
from .socketcan_backend import CANBusBackend, CANBusConfig, SocketCANBus
from .unit_conversion import (
    J1TransmissionMissingError,
    JointCalibrationTable,
    JointNodeMap,
    PerJointCalibration,
    TransmissionSpec,
    UnitConverter,
)

_JOINT_NAMES: tuple[str, ...] = DEFAULT_JOINT_NAMES
_N_JOINTS = len(_JOINT_NAMES)


class HardwareBridge(Node):
    """真机执行端：命令流 → 安全网关 → 单位换算 → CAN → 反馈 → 状态流。"""

    def __init__(
        self,
        *,
        backend: CANBusBackend | None = None,
        hardware_mode: str = "shadow",
        j1_transmission: TransmissionSpec | None = None,
        calibration: JointCalibrationTable | None = None,
        node_map: JointNodeMap | None = None,
        poll_frequency_hz: float = 100.0,
        feedback_timeout_s: float = 0.2,
        command_timeout_s: float = 0.2,
        velocity_limit: float = 3.0,
        dq_limit: float = 0.01,
    ) -> None:
        super().__init__("hardware_bridge")
        self.hardware_mode = hardware_mode

        # 节点映射与标定
        self._node_map = node_map or JointNodeMap.from_joint_list(_JOINT_NAMES)
        self._calibration = calibration or JointCalibrationTable.from_entries(
            [PerJointCalibration(j) for j in _JOINT_NAMES]
        )
        self._converter = UnitConverter(
            node_map=self._node_map,
            calibration=self._calibration,
            j1_transmission=j1_transmission,
        )

        # 安全网关
        self._watchdog = WatchdogConfig(
            feedback_timeout_s=feedback_timeout_s,
            command_timeout_s=command_timeout_s,
            qdot_limit=np.full(_N_JOINTS, velocity_limit),
            dq_per_command_limit=np.full(_N_JOINTS, dq_limit),
        )
        self._safety_gate = CommandSafetyGate(self._watchdog)
        self._recorder = ShadowCommandRecorder() if hardware_mode == "shadow" else None

        # CAN 后端
        self._backend = backend
        self._bus: SocketCANBus | None = None
        if backend is not None:
            cfg = CANBusConfig(
                node_ids=tuple(self._node_map.node_of(j) for j in _JOINT_NAMES),
            )
            self._bus = SocketCANBus(cfg, backend=backend)

        # 内部状态
        self._last_command: PositionCommand | None = None
        self._last_feedback_q: np.ndarray | None = None
        self._latest_q: np.ndarray | None = None
        self._step_count = 0
        self._fault_inject: dict = {}

        # ROS 接口
        qos = state_stream_qos()
        self._cmd_sub = self.create_subscription(
            JointState, OSCBF_COMMAND_TOPIC, self._on_command, qos)
        self._state_pub = self.create_publisher(
            JointState, JOINT_STATE_TOPIC, qos)

        # 反馈发布定时器（仅 shadow/live 模式）
        self._feedback_timer = None
        if hardware_mode != "sim":
            self._feedback_timer = self.create_timer(
                1.0 / poll_frequency_hz, self._tick_feedback)

        self.get_logger().info(
            f"hardware_bridge started: mode={hardware_mode}, "
            f"j1_ready={self._converter.is_j1_ready}, "
            f"backend={'injected' if backend is not None else 'none'}")

    # --- 命令处理 ---

    def _on_command(self, msg: JointState) -> None:
        """接收位置命令，经网关后发送 CAN 帧。"""
        if len(msg.position) != _N_JOINTS:
            self.get_logger().warn(
                f"命令关节数不匹配: {len(msg.position)} vs {_N_JOINTS}")
            return

        q_urdf = np.array(msg.position, dtype=float)
        now = time.time()

        # 构造网关命令
        cmd = PositionCommand(
            q=q_urdf, stamp_s=now,
            valid_until_s=now + self._watchdog.command_timeout_s,
            source="controller",
        )

        # 构造反馈状态（用上一次反馈或当前命令作基准）
        fb_q = self._last_feedback_q if self._last_feedback_q is not None else q_urdf
        state = HardwareState(
            q=fb_q, qdot=np.zeros(_N_JOINTS),
            stamp_s=now, feedback_ok=True,
            **self._fault_inject,
        )

        # 网关校验
        safe_cmd = self._safety_gate.evaluate(state, cmd, now_s=now)
        self._last_command = safe_cmd

        # shadow 模式只记录
        if self._recorder is not None:
            self._recorder.record(state, safe_cmd)
            return

        # live 模式：发送 CAN 帧
        if self.hardware_mode == "live" and self._bus is not None:
            self._send_can_frames(safe_cmd.q)

    def _send_can_frames(self, q_urdf: np.ndarray) -> None:
        """把关节位置换算成电机读数后发 CAN 0x19 帧。"""
        for i, joint in enumerate(_JOINT_NAMES):
            try:
                encoder_deg = self._converter.joint_to_encoder(joint, float(q_urdf[i]))
            except J1TransmissionMissingError:
                # J1 未标定：跳过（fail-closed）
                continue
            self._bus.send_position(
                self._node_map.node_of(joint), encoder_deg,
                speed=1.0, filter_accel=1.0,
            )

    # --- 反馈处理 ---

    def _tick_feedback(self) -> None:
        """定时轮询反馈并发布状态流。"""
        if self._bus is None:
            return
        states = self._bus.poll_all()
        q = np.zeros(_N_JOINTS)
        for i, joint in enumerate(_JOINT_NAMES):
            node_id = self._node_map.node_of(joint)
            state = self._bus.get_node_state(node_id)
            if state is not None and state.online:
                try:
                    q[i] = self._converter.encoder_to_joint(joint, state.pos_deg)
                except J1TransmissionMissingError:
                    q[i] = 0.0  # J1 未标定：保持零位
        self._last_feedback_q = q.copy()
        self._publish_state(q)

    def _publish_feedback_as_state(self) -> None:
        """手动触发一次反馈发布（测试用）。"""
        if self._last_feedback_q is not None:
            self._publish_state(self._last_feedback_q)

    def _publish_state(self, q: np.ndarray) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(_JOINT_NAMES)
        msg.position = [float(v) for v in q]
        self._state_pub.publish(msg)

    # --- 测试辅助 ---

    def _inject_state_fault(self, **kwargs) -> None:
        """注入故障状态（测试用：estop_active, fault_code, feedback_ok 等）。"""
        self._fault_inject.update(kwargs)

    def acknowledge_stop(self) -> None:
        """人工确认健康（清除锁存）。"""
        now = time.time()
        fb_q = self._last_feedback_q if self._last_feedback_q is not None \
            else np.zeros(_N_JOINTS)
        state = HardwareState(
            q=fb_q, qdot=np.zeros(_N_JOINTS),
            stamp_s=now, feedback_ok=True,
        )
        self._safety_gate.acknowledge_stop(state, now_s=now)

    @property
    def safety_gate(self) -> CommandSafetyGate:
        return self._safety_gate


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = HardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
