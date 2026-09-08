"""单位换算与零位偏移测试：关节↔node_id、rad↔deg、J1 丝杆换算与 fail-closed。"""

import numpy as np
import pytest

from robot_safecontrol_moveit.unit_conversion import (
    J1TransmissionMissingError,
    JointCalibrationTable,
    JointNodeMap,
    PerJointCalibration,
    TransmissionSpec,
    UnitConverter,
)


def _default_map():
    return JointNodeMap.from_joint_list(("J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"))


def _calibration(sign=1, zero_deg=0.0):
    signs = {f"J{i}": sign for i in range(1, 10)}
    return JointCalibrationTable.from_entries(
        PerJointCalibration(joint=f"J{i}", sign=signs[f"J{i}"], zero_offset_deg=zero_deg)
        for i in range(1, 10)
    )


class TestJointNodeMap:
    def test_default_mapping(self) -> None:
        m = _default_map()
        assert m.node_of("J1") == 1
        assert m.node_of("J9") == 9
        assert m.joint_of(5) == "J5"

    def test_duplicate_node_ids_rejected(self) -> None:
        with pytest.raises(ValueError):
            JointNodeMap({f"J{i}": 1 for i in range(1, 10)})

    def test_out_of_range_rejected(self) -> None:
        with pytest.raises(ValueError):
            JointNodeMap({f"J{i}": i for i in range(1, 9)} | {"J9": 64})

    def test_unknown_lookup_rejected(self) -> None:
        m = _default_map()
        with pytest.raises(KeyError):
            m.node_of("J10")
        with pytest.raises(KeyError):
            m.joint_of(0)


class TestRotaryConversion:
    def test_identity_roundtrip(self) -> None:
        conv = UnitConverter(
            node_map=_default_map(), calibration=_calibration(),
            j1_transmission=TransmissionSpec(lead_mm_per_rev=10.0),
        )
        q = np.array([0.01, 0.5, -1.2, 2.0, 3.0, 1.0, -0.4, 0.2, 0.9])
        enc = conv.joints_to_encoder(q)
        assert np.allclose(conv.encoder_to_joints(enc), q, atol=1e-9)

    def test_sign_and_offset(self) -> None:
        # J2: sign=-1, 零位偏移 5° → 读数 185° 映射 -180° (即 -π)
        table = JointCalibrationTable.from_entries([PerJointCalibration("J2", sign=-1, zero_offset_deg=5.0)])
        conv = UnitConverter(node_map=_default_map(), calibration=table)
        # J2 读数 185° → -(185-5)° = -180° = -π
        q_j2 = conv.encoder_to_joint("J2", 185.0)
        assert q_j2 == pytest.approx(-np.pi, abs=1e-9)
        assert conv.joint_to_encoder("J2", q_j2) == pytest.approx(185.0, abs=1e-6)

    def test_zero_at_offset(self) -> None:
        table = JointCalibrationTable.from_entries([PerJointCalibration("J2", zero_offset_deg=7.0)])
        conv = UnitConverter(node_map=_default_map(), calibration=table)
        assert conv.encoder_to_joint("J2", 7.0) == pytest.approx(0.0, abs=1e-12)


class TestJ1FailClosed:
    def test_uncalibrated_raises(self) -> None:
        conv = UnitConverter(node_map=_default_map(), calibration=_calibration())
        with pytest.raises(J1TransmissionMissingError):
            conv.encoder_to_joint("J1", 0.0)
        with pytest.raises(J1TransmissionMissingError):
            conv.joint_to_encoder("J1", 0.1)
        assert conv.is_j1_ready is False

    def test_calibrated_roundtrip(self) -> None:
        trans = TransmissionSpec(lead_mm_per_rev=10.0, ratio_motor_rev_per_lead_rev=1.0)
        conv = UnitConverter(
            node_map=_default_map(), calibration=_calibration(), j1_transmission=trans)
        assert conv.is_j1_ready is True
        # 电机转 1 圈 = 10 mm；0.005 m → 180°
        assert conv.encoder_to_joint("J1", 360.0) == pytest.approx(0.01, abs=1e-9)
        assert conv.joint_to_encoder("J1", 0.005) == pytest.approx(180.0, abs=1e-6)
        assert conv.encoder_to_joints(conv.joints_to_encoder(np.array([0.25] * 9)))[0] == pytest.approx(0.25, abs=1e-6)

    def test_ratio_scales(self) -> None:
        trans = TransmissionSpec(lead_mm_per_rev=10.0, ratio_motor_rev_per_lead_rev=2.0)
        conv = UnitConverter(node_map=_default_map(), calibration=_calibration(), j1_transmission=trans)
        # 电机转 2 圈才推进 10 mm → 电机转 1 圈 = 5 mm
        assert conv.encoder_to_joint("J1", 360.0) == pytest.approx(0.005, abs=1e-9)


class TestValidation:
    def test_mixed_sign_bad_value_rejected(self) -> None:
        with pytest.raises(ValueError):
            PerJointCalibration("J2", sign=2)
        with pytest.raises(ValueError):
            PerJointCalibration("J10")

    def test_duplicate_entries_rejected(self) -> None:
        with pytest.raises(ValueError):
            JointCalibrationTable.from_entries([
                PerJointCalibration("J2"), PerJointCalibration("J2")])

    def test_negative_lead_rejected(self) -> None:
        with pytest.raises(ValueError):
            TransmissionSpec(lead_mm_per_rev=-1.0)
