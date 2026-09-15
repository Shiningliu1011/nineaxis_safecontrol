"""End-to-end startup contract for perception_bridge's calibration gate.

真节点进程边界上的回归：用真参数文件起 ``perception_bridge``（隔离 ROS 域），只断言
外部可见行为——进程退出码、stderr 里的错误码、``/perception/calibration_status`` 与
``/perception/status`` 的内容。纯加载器契约在 ``tests/test_calibration_record.py``；
这里存在的理由是 fail-closed 启动是一条安全契约，手工脚本顶不住后续改动。

约定（ADR 0009 实施契约）：
* 任一拒绝条件命中 → 退出码 3，stderr 打印错误码，节点不发布任何诊断。
* 演示 profile 起飞并发布诊断；被豁免的相机是 WARN 且不可准入。
* 每帧校验 ``header.frame_id``：不符即丢弃（不用别的矩阵兜底）。

没有构建产物（install/ 不可解析）时整文件跳过，而不是失败。
"""

from __future__ import annotations

import os
from contextlib import contextmanager
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "src"))

from ament_index_python.packages import (  # noqa: E402
    PackageNotFoundError,
    get_package_share_directory,
)
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from rclpy.context import Context  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402
from sensor_msgs_py import point_cloud2 as pc2  # noqa: E402
from std_msgs.msg import Float32MultiArray, Header  # noqa: E402
import rclpy  # noqa: E402


_PACKAGE = "robot_safecontrol_moveit"
_EXECUTABLE = "perception_bridge"
_CALIBRATION_TOPIC = "/perception/calibration_status"
_STATUS_TOPIC = "/perception/status"

# fail-closed 启动契约的进程级判据（ADR 0009 决定 4）。
_REFUSAL_EXIT_CODE = 3
_ACCEPTANCE_NOT_EVALUATED = "CALIB_ACCEPTANCE_NOT_EVALUATED"

# 起真节点要构造融合引擎，给足宽限；诊断在启动末尾立即发一份，无需等心跳。
_STARTUP_TIMEOUT_S = 90.0
_POLL_PERIOD_S = 0.1
_REFUSAL_TIMEOUT_S = 60.0
_FRAME_WINDOW_S = 6.0

# /perception/status 的字段顺序见 PerceptionBridge._publish_status（10 元素浮点数组）。
_STATUS_CAMERA_USED = 4
_STATUS_SOURCE_COUNT = 8

# 与 final_launch 测试的域号错开，避免同一次会话里两个真节点测试互相看见对方的图。
_TEST_DOMAIN_ID = 92 + (os.getpid() % 20)

# 假定相机外参把相机系点映射到 workspace 内，所以「被采用」是真实可观测的。
_CAMERA_POINTS = np.column_stack([
    np.random.default_rng(7).normal(0.0, 0.05, 400),
    np.random.default_rng(8).normal(0.5, 0.05, 400),
    np.random.default_rng(9).normal(0.9, 0.05, 400),
]).astype(np.float32)


def _share_dir() -> Path | None:
    try:
        return Path(get_package_share_directory(_PACKAGE))
    except (PackageNotFoundError, LookupError, OSError):
        return None


pytestmark = pytest.mark.skipif(
    _share_dir() is None,
    reason="需要先构建 install/（colcon build --symlink-install --packages-select "
           f"{_PACKAGE}）并 source install/setup.bash",
)


def _profile(name: str) -> Path:
    return Path(_share_dir()) / "config" / name  # type: ignore[arg-type]


def _profile_doc(path: Path) -> dict:
    doc = yaml.safe_load(path.read_text(encoding="utf-8"))
    return doc["perception_bridge"]["ros__parameters"]


@contextmanager
def _running_bridge(params_file: Path, extra: tuple[str, ...] = (),
                    log_name: str = "bridge"):
    """起真节点（独立进程组）并在退出时整组收干净。

    ``ros2 run`` 会派生节点子进程：只终止包装进程会留下存活的节点，后续用例就会
    看见上一次的残留实例（诊断的 node_identity 也就不再是当前进程）。
    """
    log_path = Path(tempfile.mkdtemp(prefix="off01-bridge-")) / f"{log_name}.log"
    env = dict(os.environ, ROS_DOMAIN_ID=str(_TEST_DOMAIN_ID))
    cmd = ["ros2", "run", _PACKAGE, _EXECUTABLE,
           "--ros-args", "--params-file", str(params_file), *extra]
    with open(log_path, "w", encoding="utf-8") as log:
        proc = subprocess.Popen(
            cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        try:
            yield proc, log_path
        finally:
            _terminate(proc)


def _terminate(proc: subprocess.Popen) -> None:
    if proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=10.0)


def _run_to_exit(params_file: Path, extra: tuple[str, ...] = ()) -> subprocess.CompletedProcess:
    env = dict(os.environ, ROS_DOMAIN_ID=str(_TEST_DOMAIN_ID))
    cmd = ["ros2", "run", _PACKAGE, _EXECUTABLE,
           "--ros-args", "--params-file", str(params_file), *extra]
    return subprocess.run(cmd, env=env, capture_output=True, text=True,
                          encoding="utf-8", errors="replace",
                          timeout=_REFUSAL_TIMEOUT_S)


@contextmanager
def _probe_node():
    context = Context()
    rclpy.init(context=context, domain_id=_TEST_DOMAIN_ID)
    node = rclpy.create_node("off01_bridge_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        yield node, executor
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)


def _wait_for_message(executor, subscription_messages: list, deadline: float,
                      label: str):
    while time.monotonic() < deadline and not subscription_messages:
        executor.spin_once(timeout_sec=_POLL_PERIOD_S)
    assert subscription_messages, f"未在 {label} 收到任何消息"
    return subscription_messages[-1]


def _diagnostic_entries(msg: DiagnosticArray) -> dict[str, object]:
    return {status.name: status for status in msg.status}


class _StatusProbe:
    """订阅 /perception/status 并缓存最近一帧（Float32MultiArray 的 10 元素契约）。

    必须用 sensor-data QoS：节点用 `qos_profile_sensor_data`（BEST_EFFORT）发布，
    默认的 RELIABLE 订阅与它不兼容，会一帧都收不到（QoS 不兼容时不会报错，只在
    发现发布者时 WARN）。
    """

    def __init__(self, node):
        self.messages: list[Float32MultiArray] = []
        self._sub = node.create_subscription(
            Float32MultiArray, _STATUS_TOPIC, self.messages.append,
            qos_profile_sensor_data)

    @property
    def latest(self) -> list[float] | None:
        return list(self.messages[-1].data) if self.messages else None


# ---------------------------------------------------------------------------
# 正例：演示 profile 起飞并发布记录身份
# ---------------------------------------------------------------------------
def test_demo_profile_publishes_record_and_source_entries() -> None:
    demo = _profile("perception_runtime.yaml")
    with _running_bridge(demo, log_name="happy") as (proc, log_path):
        with _probe_node() as (node, executor):
            received: list[DiagnosticArray] = []
            node.create_subscription(
                DiagnosticArray, _CALIBRATION_TOPIC, received.append, 10)
            msg = _wait_for_message(
                executor, received, time.monotonic() + _STARTUP_TIMEOUT_S,
                "/perception/calibration_status")

        assert proc.poll() is None, (
            f"演示 profile 应保持运行，实际已退出 rc={proc.returncode}；"
            f"日志 {log_path}")

        entries = _diagnostic_entries(msg)
        record = entries["perception_bridge:record"]
        camera = entries["perception_bridge:camera"]

        # 记录身份：节点上报它实际加载的那份字节与解析到的路径。
        values = {kv.key: kv.value for kv in record.values}
        assert values["start"] == "true"
        assert values["acceptance"] == _ACCEPTANCE_NOT_EVALUATED
        assert re.fullmatch(r"[0-9a-f]{64}", values["file_sha256"])
        assert values["resolved_path"].endswith("sensor_extrinsics.yaml")
        assert values["lookup_path"], "查找路径必须上报（软链接/复制安装分叉要能看出来）"
        assert values["sources_present"].split(",") == ["camera", "lidar"]

        # 严重级字段是单字节 bytes（rclpy 的 byte 字段语义），消费端按 level[0] 读。
        for status in msg.status:
            assert isinstance(status.level, bytes) and len(status.level) == 1
        assert record.level[0] == 1, "演示 profile 带未标定豁免，记录级应为 WARN"
        assert camera.level[0] == 1

        # 实例标识：消费端据此判断这份报告属于当前进程。
        identity = re.compile(r"^perception_bridge#\d+$")
        assert identity.fullmatch(values["node_identity"])
        for status in msg.status:
            assert status.hardware_id == values["node_identity"]

        # 逐源：相机被启用、被豁免可以启动，但仍然不可准入。
        source_values = {kv.key: kv.value for kv in camera.values}
        assert source_values["enabled"] == "true"
        assert source_values["exempt"] == "true"
        assert source_values["calibrated"] == "false"
        assert source_values["math_valid"] == "true"
        assert source_values["admission_ready"] == "false"
        assert "CALIB_NOT_CALIBRATED" in source_values["issues"]
        assert source_values["time_domain"] == "unknown"
        assert source_values["time_domain_source"] == "unknown"
        assert source_values["frame_from"] == "camera_color_optical_frame"
        assert source_values["calibration_id"].startswith("v1:")

        # 记录里存在但本次未启用的源也要上报（「记录里有什么」≠「这次用了什么」）。
        lidar_values = {
            kv.key: kv.value for kv in entries["perception_bridge:lidar"].values
        }
        assert lidar_values["enabled"] == "false"
        assert lidar_values["exempt"] == "false"

        # 启动日志必须留下同一份身份，供事后对照。
        log = log_path.read_text(encoding="utf-8", errors="replace")
        assert "CALIBRATION_RECORD_LOADED" in log
        assert values["file_sha256"] in log
        assert "CALIBRATION_WARN" in log


# ---------------------------------------------------------------------------
# 反例：每一类拒绝都以退出码 3 结束并给出错误码
# ---------------------------------------------------------------------------
@pytest.fixture(scope="module")
def record_variants(tmp_path_factory) -> dict[str, Path]:
    """由发行记录派生两类坏记录（只改一处，其余逐字节相同）。

    坏记录不动声明身份：本用例要验的是数学/帧绑定的拒绝，不希望身份不自洽
    （那是 WARN 不是拒绝）混进诊断。
    """
    text = (_profile("sensor_extrinsics.yaml")).read_text(encoding="utf-8")
    out = tmp_path_factory.mktemp("records")

    bad_matrix = out / "bad_matrix.yaml"
    # 相机分节矩阵的第一个元素（文件中第一处独立的 "    - 0.0"）。
    bad_matrix.write_text(
        text.replace("\n    - 0.0\n", "\n    - 3.0\n", 1), encoding="utf-8")
    assert bad_matrix.read_text(encoding="utf-8") != text

    bad_frame = out / "bad_frame.yaml"
    bad_frame.write_text(
        text.replace("frame_from: camera_color_optical_frame",
                     "frame_from: some_other_camera"),
        encoding="utf-8")
    assert "some_other_camera" in bad_frame.read_text(encoding="utf-8")

    return {"bad_matrix": bad_matrix, "bad_frame": bad_frame}


@pytest.fixture(scope="module")
def exempt_lidar_profile(tmp_path_factory) -> Path:
    """演示 profile，但把豁免声明写成了未标定的 lidar 而不是 camera。"""
    src = _profile("perception_runtime.yaml")
    doc = yaml.safe_load(src.read_text(encoding="utf-8"))
    doc["perception_bridge"]["ros__parameters"]["allow_uncalibrated_debug"] = ["lidar"]
    out = tmp_path_factory.mktemp("profiles") / "exempt_lidar.yaml"
    out.write_text(
        yaml.safe_dump(doc, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return out


@pytest.fixture(scope="module")
def no_source_profile(tmp_path_factory) -> Path:
    """A valid record with both runtime source topics deliberately disabled."""
    src = _profile("perception_runtime.yaml")
    doc = yaml.safe_load(src.read_text(encoding="utf-8"))
    params = doc["perception_bridge"]["ros__parameters"]
    params["source_topic"] = ""
    params["source_topic_lidar"] = ""
    params.pop("allow_uncalibrated_debug", None)
    out = tmp_path_factory.mktemp("profiles") / "no_sources.yaml"
    out.write_text(
        yaml.safe_dump(doc, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return out


@pytest.fixture(scope="module")
def empty_camera_frame_profile(tmp_path_factory) -> Path:
    """An enabled camera must not be able to disable frame checks with an empty name."""
    src = _profile("perception_runtime.yaml")
    doc = yaml.safe_load(src.read_text(encoding="utf-8"))
    doc["perception_bridge"]["ros__parameters"]["input_frame"] = ""
    out = tmp_path_factory.mktemp("profiles") / "empty_camera_frame.yaml"
    out.write_text(
        yaml.safe_dump(doc, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return out


_LEGACY_MATRIX = "[0.0,0.0,1.0,-0.9,-1.0,0.0,0.0,0.4,0.0,-1.0,0.0,1.7,0.0,0.0,0.0,1.0]"


def test_legacy_extrinsics_parameter_refuses_startup() -> None:
    # 旧名字仍被声明（否则覆盖会被静默忽略），值一旦偏离默认就必须拒绝启动。
    for label, extra in (
        ("旧矩阵参数覆盖",
         ("-p", f"camera_to_world_static:={_LEGACY_MATRIX}")),
        ("旧矩阵参数写成整数数组",
         ("-p", "camera_to_world_static:=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]")),
        ("旧 TF 开关置真", ("-p", "use_tf:=true")),
    ):
        result = _run_to_exit(_profile("perception_runtime.yaml"), extra)
        combined = result.stdout + result.stderr
        assert result.returncode == _REFUSAL_EXIT_CODE, (
            f"{label}: 期望退出码 {_REFUSAL_EXIT_CODE}，实际 {result.returncode}\n{combined}")
        assert "CALIB_LEGACY_PARAM_PRESENT" in combined, f"{label}: 缺少错误码\n{combined}"


def test_missing_record_refuses_startup() -> None:
    result = _run_to_exit(_profile("perception_runtime.yaml"),
                          ("-p", "calibration_record_path:=/tmp/off01-no-such-file.yaml"))
    combined = result.stdout + result.stderr
    assert result.returncode == _REFUSAL_EXIT_CODE
    assert "CALIB_RECORD_MISSING" in combined
    assert "PERCEPTION_BRIDGE_CALIBRATION_REFUSED" in combined


def test_broken_matrix_refuses_startup(record_variants) -> None:
    result = _run_to_exit(
        _profile("perception_runtime.yaml"),
        ("-p", f"calibration_record_path:={record_variants['bad_matrix']}"))
    combined = result.stdout + result.stderr
    assert result.returncode == _REFUSAL_EXIT_CODE
    assert "CALIB_MATRIX_NOT_ORTHONORMAL" in combined


def test_frame_mismatch_refuses_startup(record_variants) -> None:
    # 绑定到别的相机的记录：即使数学合法也不能拿来给这台相机用。
    result = _run_to_exit(
        _profile("perception_runtime.yaml"),
        ("-p", f"calibration_record_path:={record_variants['bad_frame']}"))
    combined = result.stdout + result.stderr
    assert result.returncode == _REFUSAL_EXIT_CODE
    assert "CALIB_FRAME_MISMATCH" in combined


def test_empty_source_frame_refuses_startup(empty_camera_frame_profile) -> None:
    result = _run_to_exit(empty_camera_frame_profile)
    combined = result.stdout + result.stderr
    assert result.returncode == _REFUSAL_EXIT_CODE
    assert "CALIB_FRAME_MISMATCH" in combined


def test_wrong_exemption_list_refuses_startup(exempt_lidar_profile) -> None:
    # 启用的相机没有被豁免，而豁免名单写的是未被启用的 lidar。
    result = _run_to_exit(exempt_lidar_profile)
    combined = result.stdout + result.stderr
    assert result.returncode == _REFUSAL_EXIT_CODE
    assert "CALIB_SOURCE_NOT_EXEMPT" in combined


def test_deploy_profile_refuses_startup_until_calibrated() -> None:
    # 部署 profile 不设逐源豁免：真机部署不得用未标定几何起步。
    result = _run_to_exit(_profile("perception_dual_sensor_real.yaml"))
    combined = result.stdout + result.stderr
    assert result.returncode == _REFUSAL_EXIT_CODE
    assert "CALIB_SOURCE_NOT_EXEMPT" in combined


def test_refused_instance_publishes_no_diagnostic() -> None:
    """被拒绝的实例不发诊断——「收不到报告」本身是消费端可判定的信号。"""
    with _probe_node() as (node, executor):
        received: list[DiagnosticArray] = []
        node.create_subscription(
            DiagnosticArray, _CALIBRATION_TOPIC, received.append, 10)
        _run_to_exit(_profile("perception_dual_sensor_real.yaml"))
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=_POLL_PERIOD_S)
    assert not received, "被拒绝启动的实例不得发布校准诊断"


def test_no_enabled_source_starts_with_warning(no_source_profile) -> None:
    """The explicit no-source state is inert but valid, rather than an empty-topic crash."""
    with _running_bridge(no_source_profile, log_name="no-sources") as (proc, _log_path):
        with _probe_node() as (node, executor):
            received: list[DiagnosticArray] = []
            node.create_subscription(
                DiagnosticArray, _CALIBRATION_TOPIC, received.append, 10)
            msg = _wait_for_message(
                executor, received, time.monotonic() + _STARTUP_TIMEOUT_S,
                _CALIBRATION_TOPIC)
        assert proc.poll() is None, "无启用源应保持 inert 运行，而不是因空 topic 退出"
        record = _diagnostic_entries(msg)["perception_bridge:record"]
        values = {kv.key: kv.value for kv in record.values}
        assert record.level[0] == 1
        assert "no sensor source is enabled" in values["warnings"]


# ---------------------------------------------------------------------------
# 运行期帧绑定：错帧丢弃 / 对帧融合
# ---------------------------------------------------------------------------
def _run_frame_case(frame_id: str) -> tuple[list[float] | None, str]:
    demo = _profile("perception_runtime.yaml")
    camera_topic = _profile_doc(demo)["source_topic"]
    with _running_bridge(demo, log_name=f"frame-{frame_id}") as (proc, log_path):
        with _probe_node() as (node, executor):
            gate: list[DiagnosticArray] = []
            node.create_subscription(
                DiagnosticArray, _CALIBRATION_TOPIC, gate.append, 10)
            _wait_for_message(executor, gate,
                              time.monotonic() + _STARTUP_TIMEOUT_S,
                              _CALIBRATION_TOPIC)

            status = _StatusProbe(node)
            publisher = node.create_publisher(
                PointCloud2, camera_topic, qos_profile_sensor_data)
            points = _CAMERA_POINTS

            def publish() -> None:
                header = Header()
                header.frame_id = frame_id
                header.stamp = node.get_clock().now().to_msg()
                publisher.publish(pc2.create_cloud_xyz32(header, points))

            timer = node.create_timer(0.05, publish)
            # 采样必须在发布者仍在发布时进行：源有最大观测年龄，停发后再看只会得到
            # 「已超龄」的结果，看不出帧被接受还是被丢弃。
            deadline = time.monotonic() + _FRAME_WINDOW_S
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=_POLL_PERIOD_S)
            latest = status.latest
            node.destroy_timer(timer)
            node.destroy_publisher(publisher)

        assert proc.poll() is None, f"节点不应在帧绑定用例中退出（日志 {log_path}）"
        log = log_path.read_text(encoding="utf-8", errors="replace")
        return latest, log


def test_bound_frame_is_fused() -> None:
    latest, _log = _run_frame_case("camera_color_optical_frame")
    assert latest is not None, "未收到 /perception/status"
    assert latest[_STATUS_CAMERA_USED] >= 1.0, (
        f"与记录绑定一致的帧应被采用，/perception/status={latest}")
    assert latest[_STATUS_SOURCE_COUNT] >= 1.0


def test_unbound_frame_is_dropped_without_fallback() -> None:
    latest, log = _run_frame_case("wrong_camera_frame")
    assert latest is not None, "未收到 /perception/status"
    assert latest[_STATUS_CAMERA_USED] == 0.0, (
        f"错帧不得被融合（没有回退矩阵），/perception/status={latest}")
    assert latest[_STATUS_SOURCE_COUNT] == 0.0
    assert "dropping cloud whose header.frame_id" in log, (
        "错帧必须留下限流告警，否则现场无法区分「没数据」和「帧名配错」")
