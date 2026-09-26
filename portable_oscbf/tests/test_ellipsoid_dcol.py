from __future__ import annotations

import copy
import hashlib
import itertools
import json
from pathlib import Path

import jax.numpy as jnp
import mpmath as mp
import numpy as np
import pytest

from collision_parameter_inputs import DEVICE, SCENARIO, parameter_payload
from work.collision_parameters import CollisionParameterArtifact, CollisionPolicy
from work.collision_safety import (
    CollisionIdentities, CollisionSafety, CollisionSafetyConfig, CollisionScene,
    CollisionStatus, QueryBatch, QueryMode,
)
from work.nineaxis_kinematics import NineaxisKinematics


def _geometry(links, centers, radii, capacity=1):
    geometry = {
        "schema_version": 1, "slot_capacity": capacity,
        "source": "analytic ellipsoids for numerical validation; no mesh coverage claim",
        "links": [
            {"link": name, "slots": [
                {"center_m": list(center), "radii_m": list(radius), "active_mask": True},
                *[{"center_m": [0.0]*3, "radii_m": [1.0]*3, "active_mask": False}
                  for _ in range(capacity - 1)],
            ]} for name, center, radius in zip(links, centers, radii)
        ],
    }
    document = {
        "geometry": geometry,
        "self_collision": {"candidate_pairs": list(map(list, itertools.combinations(links, 2))),
                           "allowed_contacts": []},
    }
    _sign_geometry(document)
    return document


def _sign_geometry(document):
    encoded = json.dumps(document["geometry"], sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    document["geometry_hash"] = hashlib.sha256(encoded).hexdigest()


def _module(document, clearance_mm=1.0, **limits):
    payload = parameter_payload()
    payload["parameters"]["max_ellipsoids_per_link"]["value"] = document["geometry"]["slot_capacity"]
    for name, value in {
        "dcol_max_iterations": 80, "dcol_primal_residual_limit": 1e-10,
        "dcol_dual_residual_limit": 1e-10, **limits,
    }.items():
        payload["parameters"][name]["value"] = value
    clearance = copy.deepcopy(payload["self_clearance_mm"][0]["clearance"])
    clearance["value"] = clearance_mm
    payload["self_clearance_mm"] = [
        {"links": pair, "clearance": copy.deepcopy(clearance)}
        for pair in document["self_collision"]["candidate_pairs"]
    ]
    artifact = CollisionParameterArtifact.create(payload, device=DEVICE, scenario=SCENARIO)
    policy = CollisionPolicy(artifact, bytes.fromhex(document["geometry_hash"]), b"d" * 32,
                             tuple(document["self_collision"]["allowed_contacts"]))
    return CollisionSafety(CollisionSafetyConfig.from_policy(policy), geometry_artifact=document)


def _prepared(module, status=CollisionStatus.OK, occupied=False):
    config = module.config
    scene = CollisionScene(
        jnp.zeros((3, 3)), jnp.zeros(3), jnp.full(3, 30.0), jnp.zeros((3, 3)),
        jnp.arange(3, dtype=jnp.int32), jnp.array([occupied, False, False]),
        jnp.int64(100), jnp.int64(120), jnp.int32(status),
    )
    identities = CollisionIdentities(
        jnp.ones(16, dtype=jnp.uint8), jnp.int64(1),
        *[jnp.asarray(np.frombuffer(getattr(config, field), dtype=np.uint8))
          for field in ("geometry_hash", "kernel_version", "collision_policy_hash")],
    )
    return module.prepare_scene(scene, identities)


def _query(module, q=None, mode=QueryMode.OSCBF_BARRIER, **scene_args):
    q = np.zeros((2, 9)) if q is None else np.asarray(q)
    if q.shape == (9,):
        q = np.array([q, q])
    return module.query(
        QueryBatch(jnp.asarray(q), jnp.array([True, False])), _prepared(module, **scene_args), mode
    )


def _world(document, q):
    poses = NineaxisKinematics().forward_kinematics(np.asarray(q))
    result = []
    for link in document["geometry"]["links"]:
        pose = poses[link["link"]]
        slot = link["slots"][0]
        result.append((pose[:3, :3] @ np.array(slot["center_m"]) + pose[:3, 3],
                       pose[:3, :3] @ np.diag(slot["radii_m"])))
    return result


def _reference(world):
    # 独立求解原始五维 KKT；80 位运算和 Newton 方法均独立于 JAX 一维二分。
    with mp.workdps(80):
        (ca, la), (cb, lb) = [(mp.matrix(c.tolist()), mp.matrix(l.tolist())) for c, l in world]
        pa, pb = (la * la.T)**-1, (lb * lb.T)**-1

        def equations(x, y, z, square, weight):
            da, db = mp.matrix([x, y, z]) - ca, mp.matrix([x, y, z]) - cb
            stationarity = weight * pa * da + (1-weight) * pb * db
            return (*stationarity, (da.T * pa * da)[0] - square,
                    (db.T * pb * db)[0] - square)

        center = (ca + cb) / 2
        root = mp.findroot(equations, (*center, mp.mpf(1), mp.mpf("0.5")),
                           tol=mp.mpf("1e-65"), maxsteps=100)
        assert 0 < root[4] < 1
        assert root[3] > 0
        return float(mp.sqrt(root[3]))


@pytest.mark.parametrize("distance", [0.03, 0.1, 0.100000001, 0.099999999, 0.3])
@pytest.mark.parametrize("radii", [([0.04]*3, [0.06]*3), ([0.04, 0.02, 0.01], [0.06, 0.09, 0.03])])
def test_query_analytic_scale_and_pair_specific_margin(distance, radii):
    doc = _geometry(["base_link", "Link1"], [[0, 0, 0], [distance, 0, 0]], radii)
    module = _module(doc, clearance_mm=2.0)
    result = _query(module)
    assert result.solver_healthy[0, 0]
    assert float(result.proximity_scale[0, 0]) == pytest.approx(distance / 0.1, abs=2e-8, rel=0)
    margin = 1 + 0.002 / (min(radii[0]) + min(radii[1]))
    assert float(result.barrier[0, 0]) == pytest.approx(distance / 0.1 - margin, abs=2e-8, rel=0)
    assert result.solver_primal_residual[0, 0] <= 1e-10
    assert result.solver_dual_residual[0, 0] <= 1e-10
    assert 0 <= result.solver_iteration[0, 0] <= 80


@pytest.mark.parametrize("radii", [([1e-5]*3, [0.3]*3), ([0.04, 0.003, 0.09], [0.01, 0.03, 0.06])])
def test_query_rotated_scale_and_joint_gradient_against_independent_reference(radii):
    doc = _geometry(["base_link", "Link9"], [[0.12, 0.07, -0.01], [0.04, -0.02, 0.01]], radii)
    module = _module(doc)
    q = np.array([0.15, 0.1, -0.2, 0.3, 0.15, -0.25, 0.2, 0.05, -0.1])
    result = _query(module, q)
    assert result.solver_healthy[0, 0]
    assert float(result.proximity_scale[0, 0]) == pytest.approx(_reference(_world(doc, q)), abs=2e-8, rel=0)
    for joint in range(9):
        step = np.zeros(9)
        step[joint] = 1e-6
        difference = (_reference(_world(doc, q + step)) - _reference(_world(doc, q - step))) / 2e-6
        assert float(result.grad_h_q[0, 0, joint]) == pytest.approx(difference, abs=2e-5, rel=0)


def test_query_nearly_degenerate_ellipsoids():
    doc = _geometry(["base_link", "Link9"], [[0.12, 0.07, -0.01], [0.04, -0.02, 0.01]],
                    [[0.1, 0.0001, 0.03], [0.001, 0.03, 0.02]])
    module = _module(doc)
    q = np.linspace(0.01, 0.2, 9)
    result = _query(module, q)
    assert result.solver_healthy[0, 0]
    assert float(result.proximity_scale[0, 0]) == pytest.approx(_reference(_world(doc, q)), abs=2e-6, rel=0)
    for joint in range(9):
        step = np.zeros(9)
        step[joint] = 1e-6
        difference = (_reference(_world(doc, q + step)) - _reference(_world(doc, q - step))) / 2e-6
        assert float(result.grad_h_q[0, 0, joint]) == pytest.approx(difference, abs=2e-5, rel=0)


def test_query_unconverged_and_concentric_are_unhealthy():
    for center, limit in (([0.2, 0.1, 0.3], 1), ([0, 0, 0], 80)):
        doc = _geometry(["base_link", "Link1"], [[0, 0, 0], center], [[0.03]*3, [0.1]*3])
        result = _query(_module(doc, dcol_max_iterations=limit))
        assert not result.solver_healthy[0, 0]
        assert result.solver_iteration[0, 0] <= limit
        assert int(result.header.status) in (CollisionStatus.SOLVER_UNHEALTHY, CollisionStatus.DEADLINE_MISSED)
        assert not np.any(result.valid_mask)
        assert not np.any(result.state_valid_mask)
        if limit == 1:
            assert result.solver_primal_residual[0, 0] > 1e-10 or result.solver_dual_residual[0, 0] > 1e-10
        else:
            assert result.proximity_scale[0, 0] == 0


def test_query_masks_modes_deadline_and_unimplemented_environment():
    doc = _geometry(["base_link", "Link1"], [[0, 0, 0], [0.2, 0, 0]], [[0.03]*3, [0.03]*3], capacity=2)
    module = _module(doc)
    _query(module)
    result = _query(module, mode=QueryMode.STATE_VALIDITY)
    assert int(result.header.status) == CollisionStatus.OK
    assert np.array_equal(result.primitive_pair_id[0, 0], [0, 2])
    assert np.array_equal(result.valid_mask, [[True, False, False, False], [False]*4])
    assert np.array_equal(result.state_valid, [True, False])
    for mode, occupied in ((QueryMode.DISTANCE_MM, False), (QueryMode.OSCBF_BARRIER, True)):
        invalid = _query(module, mode=mode, occupied=occupied)
        assert int(invalid.header.status) != CollisionStatus.OK
        assert not np.any(invalid.valid_mask)
        assert not np.any(invalid.distance_valid_mask)
    late = _query(_module(doc, query_deadline_ns=1))
    assert int(late.header.status) == CollisionStatus.DEADLINE_MISSED
    assert not np.any(late.valid_mask)
    assert late.header.completed_ns - late.header.started_ns == late.header.runtime_ns


def test_query_production_geometry_retains_all_self_pairs():
    path = Path(__file__).resolve().parents[1] / "config/collision_geometry_artifact.json"
    doc = json.loads(path.read_text())
    module = _module(doc, max_primitive_rows=45)
    result = _query(module)
    assert result.proximity_scale.shape == (2, 45)
    assert np.all(result.solver_healthy[0])
    assert len(set(map(tuple, np.asarray(result.primitive_pair_id[0])))) == 45
    assert not np.any(result.solver_healthy[1])


def test_geometry_identity_complete_pairs_and_capacity_are_checked():
    doc = _geometry(["base_link", "Link1"], [[0, 0, 0], [0.2, 0, 0]], [[0.03]*3]*2)
    module = _module(doc)
    changed = copy.deepcopy(doc)
    changed["geometry"]["links"][0]["slots"][0]["radii_m"][0] = 0.1
    with pytest.raises(ValueError, match="geometry_hash"):
        CollisionSafety(module.config, geometry_artifact=changed)
    changed = copy.deepcopy(doc)
    changed["self_collision"]["candidate_pairs"] = []
    with pytest.raises(ValueError, match="candidate_pairs"):
        CollisionSafety(module.config, geometry_artifact=changed)
    doc = _geometry(["base_link", "Link1", "Link2"], [[0, 0, 0]]*3, [[0.03]*3]*3)
    with pytest.raises(ValueError, match="max_primitive_rows"):
        _module(doc, max_primitive_rows=2)


def test_query_symmetric_dangers_keep_opposing_primitive_gradients():
    doc = _geometry(["base_link", "Link1"], [[0, 0, -0.2], [0, 0, 0]], [[0.05]*3]*2, capacity=2)
    doc["geometry"]["links"][0]["slots"][1] = {
        "center_m": [0, 0, 0.2], "radii_m": [0.05]*3, "active_mask": True,
    }
    _sign_geometry(doc)
    module = _module(doc, clearance_mm=150.0)
    _query(module)
    result = _query(module)
    assert int(result.header.status) == CollisionStatus.OK
    assert np.array_equal(result.valid_mask[0], [True, True, False, False])
    assert np.allclose(result.barrier[0, :2], [-0.5, -0.5])
    assert np.allclose(result.grad_h_q[0, :2, 0], [10.0, -10.0])
    assert result.state_valid_mask[0] and not result.state_valid[0]


def test_query_active_batch_states_and_invalid_scene():
    doc = _geometry(["base_link", "Link1"], [[0, 0, 0], [0, 0, 0.15]], [[0.05]*3]*2)
    module = _module(doc)
    q = np.zeros((2, 9))
    q[1, 0] = -0.1
    batch = QueryBatch(jnp.asarray(q), jnp.array([True, True]))
    prepared = _prepared(module)
    module.query(batch, prepared, QueryMode.OSCBF_BARRIER)
    result = module.query(batch, prepared, QueryMode.OSCBF_BARRIER)
    assert int(result.header.status) == CollisionStatus.OK
    assert np.allclose(result.proximity_scale[:, 0], [1.5, 0.5])
    assert np.array_equal(result.state_valid, [True, False])
    assert np.all(result.state_valid_mask)
    for status in CollisionStatus:
        if status == CollisionStatus.OK:
            continue
        invalid = module.query(batch, _prepared(module, status=status), QueryMode.STATE_VALIDITY)
        assert int(invalid.header.status) in (status, CollisionStatus.DEADLINE_MISSED)
        assert not np.any(invalid.valid_mask)
    foreign = prepared._replace(identities=prepared.identities._replace(kernel_version=jnp.zeros(32, dtype=jnp.uint8)))
    assert int(module.query(batch, foreign, QueryMode.STATE_VALIDITY).header.status) != CollisionStatus.OK


@pytest.mark.parametrize("field,value", [("radii_m", [0.0, 0.1, 0.1]), ("radii_m", [-0.1]*3),
                                         ("center_m", [0.0, 0.0]), ("active_mask", 1)])
def test_invalid_ellipsoid_input_is_rejected(field, value):
    doc = _geometry(["base_link", "Link1"], [[0, 0, 0], [0.2, 0, 0]], [[0.03]*3]*2)
    doc["geometry"]["links"][0]["slots"][0][field] = value
    _sign_geometry(doc)
    with pytest.raises(ValueError, match="geometry slot"):
        _module(doc)


def test_nonfinite_calculation_is_unhealthy():
    doc = _geometry(["base_link", "Link1"], [[0, 0, 0], [0.2, 0, 0]], [[0.03]*3]*2)
    q = np.zeros((2, 9))
    q[0, 0] = 1e308
    result = _query(_module(doc), q)
    assert not result.solver_healthy[0, 0]
    assert int(result.header.status) != CollisionStatus.OK
    assert not np.any(result.valid_mask)


def test_nonfinite_scale_margin_is_rejected_before_query():
    doc = _geometry(["base_link", "Link1"], [[0, 0, 0], [0.002, 0, 0]], [[0.0001]*3]*2)
    with pytest.raises(ValueError, match="finite scale_margin"):
        _module(doc, clearance_mm=1e308)


def test_seeded_rotated_corpus_matches_high_precision_reference():
    doc = _geometry(["base_link", "Link9"], [[0.12, 0.07, -0.01], [0.04, -0.02, 0.01]],
                    [[0.04, 0.02, 0.08], [0.015, 0.07, 0.05]])
    module = _module(doc)
    rng = np.random.default_rng(107)
    for _ in range(12):
        q = rng.uniform(-0.5, 0.5, size=9)
        q[0] = rng.uniform(0.0, 0.5)
        result = _query(module, q)
        assert result.solver_healthy[0, 0]
        assert float(result.proximity_scale[0, 0]) == pytest.approx(_reference(_world(doc, q)), abs=2e-8, rel=0)


def test_geometry_requires_matching_clearance_contacts_and_copied_data():
    doc = _geometry(["base_link", "Link1"], [[0, 0, 0], [0.2, 0, 0]], [[0.03]*3]*2)
    original = copy.deepcopy(doc)
    module = _module(doc)
    doc["geometry"]["links"][1]["slots"][0]["center_m"] = [0, 0, 0]
    assert float(_query(module).proximity_scale[0, 0]) == pytest.approx(0.2 / 0.06)
    _sign_geometry(doc)
    payload = module.config.policy.artifact.to_document()
    payload.pop("parameter_hash")
    payload["self_clearance_mm"][0]["links"] = ["base_link", "Link2"]
    artifact = CollisionParameterArtifact.create(payload, device=DEVICE, scenario=SCENARIO)
    policy = CollisionPolicy(artifact, bytes.fromhex(doc["geometry_hash"]), b"d"*32)
    with pytest.raises(ValueError, match="self_clearance_mm"):
        CollisionSafety(CollisionSafetyConfig.from_policy(policy), geometry_artifact=doc)
    original["self_collision"]["allowed_contacts"] = [{"links": ["base_link", "Link1"]}]
    with pytest.raises(ValueError, match="allowed_contacts"):
        CollisionSafety(module.config, geometry_artifact=original)


@pytest.mark.parametrize("case", json.loads(
    (Path(__file__).parent / "reference/ellipsoid_dcol/official.json").read_text()
)["cases"], ids=lambda case: case["name"])
def test_query_matches_official_julia_dcol(case):
    doc = _geometry(case["links"], case["centers"], case["radii"])
    result = _query(_module(doc), np.array(case["q"]))
    assert result.solver_healthy[0, 0]
    tolerance = 2e-6 if case["name"] == "near-degenerate" else 2e-8
    assert float(result.proximity_scale[0, 0]) == pytest.approx(case["proximity_scale"], abs=tolerance, rel=0)
    assert _reference(_world(doc, case["q"])) == pytest.approx(case["proximity_scale"], abs=tolerance, rel=0)
