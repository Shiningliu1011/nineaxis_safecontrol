import os
from pathlib import Path

import numpy as np
import pytest
from scipy.integrate import quad
from scipy.interpolate import BSpline, PPoly
from scipy.io import loadmat
from work.continuous_path_reference import (
    ContinuousPathReference,
    curve_jet,
    time_derivatives,
)
from work.path_following import PathGeometry

ROOT = Path(__file__).resolve().parents[2]
SOURCE = Path(os.environ.get("BUTTERFLY_SOURCE_DIR", ROOT / "data/nurbs/butterfly_reference_v1/production/source"))


@pytest.fixture(scope="module")
def reference_and_source():
    control = np.loadtxt(SOURCE / "control_points.txt")
    knots = np.loadtxt(SOURCE / "knot_vector.txt")
    spline = BSpline(knots, control, 3)
    breaks = np.unique(knots)
    indices = np.searchsorted(knots, breaks[:-1], side="right") - 1
    pieces = np.stack([PPoly.from_spline((knots, control[:, axis], 3)).c[:, indices].T
                       for axis in range(3)], axis=-1)
    transform = np.eye(4)
    transform[:3, :3] *= .967379812532678
    transform[:3, 3] = [-.00004538030229213852, .33200184706543157, 1.3668826946321033]
    pieces = pieces @ transform[:3, :3].T / 1000
    pieces[:, -1] += transform[:3, 3]
    axes = np.array([[0., 1., 0.], [0., 1., 0.]])
    centres = np.array([[-.00007246874682720339, .3466393505935161, 1.6372593390674268],
                        [-.00007246874682720339, .3466393505935161, 1.6372593390674266]])
    radii = np.array([.2706009973196931, .2706009973196956])
    raw = loadmat(ROOT / "data/nurbs/ik_input.mat", simplify_cells=True)["ik_input"]
    reference = ContinuousPathReference.build(
        breaks, pieces, axes, centres, radii, raw["u_series"], raw["time_series"],
        raw["feedrate_cmd_series"] * .967379812532678 / 1000 * 3.5,
    )
    return reference, spline, raw, transform, axes, centres, radii


def independent_position(spline, u, transform, axes, centres, radii):
    points = spline(u) @ transform[:3, :3].T / 1000 + transform[:3, 3]
    for axis, centre, radius in zip(axes, centres, radii):
        relative = points - centre
        axial = np.sum(relative * axis, axis=-1, keepdims=True) * axis
        radial = relative - axial
        points = centre + axial + radius * radial / np.linalg.norm(radial, axis=-1, keepdims=True)
    return points


def test_original_butterfly_full_projection_and_final_arc(reference_and_source):
    reference, spline, raw, transform, axes, centres, radii = reference_and_source
    u = raw["u_series"]
    result = reference.query_u(u)
    expected = independent_position(spline, u, transform, axes, centres, radii)
    np.testing.assert_allclose(result.position_m, expected, atol=8e-16, rtol=0)
    assert reference.total_length_m == pytest.approx(1.8167657348637825, abs=3e-10)
    radial = expected - centres[-1]
    radial -= (radial @ axes[-1])[:, None] * axes[-1]
    np.testing.assert_allclose(result.tool_axis, -radial / np.linalg.norm(radial, axis=1)[:, None], atol=2e-15)
    assert np.all(result.valid)
    assert result.s[0] == 0 and result.s[-1] == 1
    np.testing.assert_allclose(np.linalg.norm(result.position_d1, axis=1), reference.total_length_m, atol=2e-14)


def test_realtime_and_jax_consume_the_continuous_curve(reference_and_source):
    import jax
    import jax.numpy as jnp
    from work.jax_path_following import as_jax_path_geometry, sample_path_jax

    jax.config.update("jax_enable_x64", True)
    reference = reference_and_source[0]
    geometry = PathGeometry.from_continuous_reference(reference)
    jgeometry = as_jax_path_geometry(geometry)
    compiled = jax.jit(sample_path_jax)
    for s in (0., .123456789, .50001, .987654321, 1.):
        exact = reference.query(s)
        host = geometry.sample(s * reference.total_length_m)
        device = compiled(jgeometry, jnp.asarray(s * reference.total_length_m))
        np.testing.assert_allclose(host.position_m, exact.position_m, atol=2e-14, rtol=0)
        np.testing.assert_allclose(device.position_m, exact.position_m, atol=2e-14, rtol=0)
        np.testing.assert_allclose(device.rotation[:, 0], exact.tool_axis, atol=2e-14, rtol=0)
        np.testing.assert_allclose(device.tangent, exact.position_d1 / reference.total_length_m, atol=2e-13)
    assert geometry.total_length_m == reference.total_length_m


@pytest.mark.parametrize("close_samples", [False, True])
def test_compacted_continuous_arrays_share_the_sampled_source_indices(reference_and_source, close_samples):
    original = reference_and_source[0]
    data = original.arrays
    reference = original
    if close_samples:
        source_u = np.r_[data.source_u[:1], data.source_u[1] * np.array([0., 1e-6, 2e-6]), data.source_u[1:]]
        source_time = np.r_[data.source_time_s[:1], data.source_time_s[0]
                            + np.diff(data.source_time_s[:2])[0] * np.array([.1, .2, .3]), data.source_time_s[1:]]
        source_feedrate = np.r_[data.source_feedrate_m_s[:1], np.repeat(data.source_feedrate_m_s[0], 3),
                                data.source_feedrate_m_s[1:]]
        reference = ContinuousPathReference.build(
            data.breaks_u, data.coefficients_m, data.projection_axes, data.projection_centres_m,
            data.projection_radii_m, source_u, source_time, source_feedrate)
    samples = reference.query_u(reference.arrays.source_u)
    sampled = PathGeometry.from_samples(samples.position_m, samples.rotation,
                                        reference.arrays.source_feedrate_m_s, reference.arrays.source_time_s)
    geometry = PathGeometry.from_continuous_reference(reference, identity="butterfly-compaction")
    indices = np.searchsorted(reference.arrays.source_time_s, sampled.source_time_s)
    assert len(indices) < len(samples.u)
    if close_samples:
        assert np.all(np.linalg.norm(samples.position_m[1:4] - samples.position_m[0], axis=1) < 1e-7)
        assert not np.isin([1, 2, 3], indices).any()
    for name in ("positions_m", "rotations", "quaternions_xyzw", "feedrate_m_s", "source_time_s"):
        np.testing.assert_array_equal(getattr(geometry, name), getattr(sampled, name))
    np.testing.assert_array_equal(geometry.positions_m, samples.position_m[indices])
    np.testing.assert_array_equal(geometry.rotations, samples.rotation[indices])
    np.testing.assert_array_equal(geometry.arc_length_m, samples.s[indices] * reference.total_length_m)
    np.testing.assert_array_equal(geometry.tangents, samples.position_d1[indices] / reference.total_length_m)
    np.testing.assert_array_equal(geometry.omega_per_m,
                                  np.cross(samples.tool_axis[indices], samples.axis_d1[indices]) / reference.total_length_m)
    assert geometry.continuous_reference is reference
    endpoint = geometry.continuous_reference.query(1.)
    np.testing.assert_array_equal(endpoint.source_index_interval, [len(samples.u) - 2, len(samples.u) - 1])
    assert endpoint.source_time_interval_s[1] > endpoint.source_time_interval_s[0]
    assert geometry.sample(geometry.total_length_m).source_time_s == reference.arrays.source_time_s[-1]
    np.testing.assert_array_equal(reference.query_u(data.breaks_u).event_interval,
                                  original.query_u(data.breaks_u).event_interval)


def test_projection_uses_continuous_geometry_and_an_ordered_interval(reference_and_source):
    reference = reference_and_source[0]
    exact = reference.query(.456789)
    projected = reference.project(exact.position_m + 3e-4 * exact.normal, s_interval=(.45, .46))
    assert projected["s"] == pytest.approx(.456789, abs=3e-9)
    assert projected["distance_m"] == pytest.approx(3e-4, abs=3e-12)
    bounded = reference.project(reference.query(.44).position_m, s_interval=(.45, .46))
    assert bounded["s"] == pytest.approx(.45, abs=2e-12)
    geometry = PathGeometry.from_continuous_reference(reference)
    anchor = int(np.searchsorted(geometry.arc_length_m, exact.s * reference.total_length_m))
    progress, _ = geometry.project_local(exact.position_m, anchor_segment=anchor, half_window_segments=5)
    assert progress / reference.total_length_m == pytest.approx(.456789, abs=2e-10)


def test_independent_autodiff_checks_all_parameter_derivatives(reference_and_source):
    import jax
    import jax.numpy as jnp

    jax.config.update("jax_enable_x64", True)
    reference, _, _, _, axes, centres, radii = reference_and_source
    arrays = reference.arrays
    coefficients = jnp.asarray(arrays.coefficients_m)
    breaks = jnp.asarray(arrays.breaks_u)

    def independent(u):
        index = jnp.clip(jnp.searchsorted(breaks, u, side="right") - 1, 0, len(breaks) - 2)
        h, c = u - breaks[index], coefficients[index]
        point = c[0] * h**3 + c[1] * h**2 + c[2] * h + c[3]
        for axis, centre, radius in zip(axes, centres, radii):
            relative = point - centre
            axial = jnp.dot(relative, axis) * axis
            radial = relative - axial
            point = centre + axial + radius * radial / jnp.linalg.norm(radial)
        radial = point - centres[-1]
        radial = radial - jnp.dot(radial, axes[-1]) * axes[-1]
        return jnp.concatenate((point, -radial / jnp.linalg.norm(radial)))

    functions = [independent]
    for _ in range(3):
        functions.append(jax.jacfwd(functions[-1]))
    u = (arrays.breaks_u[:-1] + arrays.breaks_u[1:]) / 2
    independent_jets = [np.asarray(jax.jit(jax.vmap(function))(jnp.asarray(u))) for function in functions]
    actual = curve_jet(arrays, u)
    for order in range(4):
        np.testing.assert_allclose(actual[0][order], independent_jets[order][:, :3], atol=2e-7, rtol=2e-11)
        np.testing.assert_allclose(actual[1][order], independent_jets[order][:, 3:], atol=5e-7, rtol=2e-11)


def test_arc_inversion_nodes_sides_and_difference_convergence(reference_and_source):
    reference = reference_and_source[0]
    arrays = reference.arrays
    u = np.r_[arrays.breaks_u, np.linspace(.001, .999, 100)]
    sample = reference.query_u(u)
    inverse = reference.query(sample.s)
    np.testing.assert_allclose(inverse.u, u, atol=8e-14, rtol=0)
    np.testing.assert_allclose(inverse.position_m, sample.position_m, atol=2e-13, rtol=0)
    independent_lengths = []
    for low, high in zip(arrays.breaks_u[:-1], arrays.breaks_u[1:]):
        independent_lengths.append(quad(lambda x: np.linalg.norm(curve_jet(arrays, np.asarray(x))[0][1]),
                                         low, high, epsabs=1e-11, epsrel=1e-11)[0])
    assert sum(independent_lengths) == pytest.approx(reference.total_length_m, abs=2e-11)
    nodes = arrays.breaks_u[1:-1]
    left, right = reference.query_u(nodes, side="left"), reference.query_u(nodes, side="right")
    for name in ("position_m", "tool_axis", "position_d1", "axis_d1", "position_d2", "axis_d2"):
        np.testing.assert_allclose(getattr(left, name), getattr(right, name), atol=3e-8, rtol=2e-10)
    assert np.max(np.linalg.norm(left.position_d3 - right.position_d3, axis=1)) > 1
    selected = reference.query(reference.query_u(nodes).s)
    np.testing.assert_allclose(selected.position_d3, right.position_d3, atol=5e-7, rtol=5e-11)
    probes = reference.query_u((arrays.breaks_u[:-1] + arrays.breaks_u[1:]) / 2)
    errors = []
    # 三阶差分使用能够区分截断误差和浮点消减误差的步长。
    for h in (1e-4, 5e-5):
        values = [reference.query(probes.s + k * h).position_m for k in (-2, -1, 0, 1, 2)]
        fmm, fm, f0, fp, fpp = values
        differences = ((fmm - 8 * fm + 8 * fp - fpp) / (12 * h),
                       (-fpp + 16 * fp - 30 * f0 + 16 * fm - fmm) / (12 * h**2),
                       (fpp - 2 * fp + 2 * fm - fmm) / (2 * h**3))
        errors.append([np.max(np.abs(value - target)) for value, target in
                       zip(differences, (probes.position_d1, probes.position_d2, probes.position_d3))])
    assert errors[-1][0] < 2e-7
    assert errors[-1][1] < 8e-4
    assert errors[-1][2] < errors[0][2] * .6


def test_pause_time_derivatives_source_hold_and_input_rejection(reference_and_source):
    reference = reference_and_source[0]
    sample = reference.query(.5)
    for values in time_derivatives(sample, 0.).values():
        for value in values:
            np.testing.assert_array_equal(value, np.zeros(3))
    motion = time_derivatives(sample, .2, -.03, .004)
    np.testing.assert_allclose(motion["position"][2], sample.position_d3 * .2**3
                               + 3 * sample.position_d2 * .2 * -.03 + sample.position_d1 * .004)
    endpoint = reference.query(1.)
    assert endpoint.source_time_interval_s[1] > endpoint.source_time_interval_s[0]
    for invalid in (-1e-10, 1 + 1e-10, np.nan, np.inf, 1j):
        with pytest.raises(ValueError):
            reference.query(invalid)
        with pytest.raises(ValueError):
            reference.query_u(invalid)
    with pytest.raises(ValueError):
        reference.query(.5, side="invalid")


def test_jax_inverse_derivatives_at_source_nodes_and_validity(reference_and_source):
    import jax
    import jax.numpy as jnp
    from work.continuous_path_reference import ContinuousReferenceArrays
    from work.jax_continuous_reference import query_reference_jax

    jax.config.update("jax_enable_x64", True)
    reference = reference_and_source[0]
    data = ContinuousReferenceArrays(*(jnp.asarray(value) for value in reference.arrays))
    derivative = jax.jit(jax.jacfwd(lambda s: query_reference_jax(data, s).position_m))
    for u in reference.arrays.breaks_u[1:-1:4]:
        expected = reference.query_u(u)
        np.testing.assert_allclose(derivative(jnp.asarray(expected.s)), expected.position_d1, atol=3e-12, rtol=0)
    invalid = jax.jit(query_reference_jax)(data, jnp.asarray(-.01))
    assert not bool(invalid.valid)


@pytest.mark.parametrize("side", ["left", "right"])
def test_jax_three_derivatives_keep_the_selected_source_node_side(reference_and_source, side):
    import jax
    import jax.numpy as jnp
    from work.continuous_path_reference import ContinuousReferenceArrays
    from work.jax_continuous_reference import query_reference_jax

    jax.config.update("jax_enable_x64", True)
    reference = reference_and_source[0]
    data = ContinuousReferenceArrays(*(jnp.asarray(value) for value in reference.arrays))
    progress = reference.query_u(reference.arrays.breaks_u).s
    expected = reference.query(progress, side=side)

    def evaluate(s):
        sample = query_reference_jax(data, s, side=side)
        return jnp.concatenate((sample.position_m, sample.tool_axis))

    derivative = evaluate
    for order in range(1, 4):
        derivative = jax.jacfwd(derivative)
        actual = np.asarray(jax.jit(jax.vmap(derivative))(jnp.asarray(progress)))
        target = np.concatenate((getattr(expected, f"position_d{order}"),
                                 getattr(expected, f"axis_d{order}")), axis=1)
        np.testing.assert_allclose(actual, target, atol=2e-8, rtol=2e-12)


def test_continuous_kernel_snapshot_contains_the_actual_coefficients(reference_and_source):
    from work.jax_control_facade import JaxControlLoop
    from work.path_following import PathFollowingConfig

    geometry = PathGeometry.from_continuous_reference(reference_and_source[0])
    loop = JaxControlLoop(enable_x64=True)
    loop.configure_path(geometry, PathFollowingConfig())
    arrays = loop.path_geometry_arrays()
    np.testing.assert_array_equal(arrays["continuous_coefficients_m"], geometry.continuous_reference.arrays.coefficients_m)
    arrays["continuous_coefficients_m"][0, 0, 0] = 123.
    assert loop.path_geometry_arrays()["continuous_coefficients_m"][0, 0, 0] != 123.


def test_numpy_recording_roundtrip_keeps_continuous_geometry_without_pickle(reference_and_source, tmp_path):
    geometry = PathGeometry.from_continuous_reference(reference_and_source[0], identity="butterfly-evidence")
    saved = tmp_path / "path.npz"
    np.savez_compressed(saved, **geometry.to_arrays())
    with np.load(saved, allow_pickle=False) as archive:
        restored = PathGeometry.from_arrays({name: archive[name] for name in archive.files})
    assert restored.reference_identity == "butterfly-evidence"
    assert restored.continuous_reference is not None
    for s in (0., .1734, .502, 1.):
        np.testing.assert_array_equal(restored.sample(s * geometry.total_length_m).position_m,
                                      geometry.sample(s * geometry.total_length_m).position_m)


@pytest.mark.parametrize("changed", [
    "positions_m", "rotations", "quaternions_xyzw", "arc_length_m", "tangents", "omega_per_m",
    "feedrate_m_s", "source_time_s", "continuous_gauss_nodes", "continuous_gauss_weights",
    "continuous_source_feedrate_m_s", "missing_continuous", "reference_identity",
])
def test_recording_rejects_inconsistent_geometry_and_reference_identity(reference_and_source, tmp_path, changed):
    geometry = PathGeometry.from_continuous_reference(reference_and_source[0], identity="butterfly-evidence")
    values = {name: np.array(value, copy=True) for name, value in geometry.to_arrays().items()}
    if changed == "missing_continuous":
        values = {name: value for name, value in values.items() if not name.startswith("continuous_")}
    elif changed == "reference_identity":
        values[changed] = np.asarray("different-reference")
    elif changed == "arc_length_m":
        values[changed] *= .5
    else:
        values[changed].flat[0] += .001
    saved = tmp_path / "path.npz"
    np.savez_compressed(saved, **values)
    with np.load(saved, allow_pickle=False) as archive, pytest.raises(ValueError):
        PathGeometry.from_arrays({name: archive[name] for name in archive.files})


def test_legacy_sampled_recording_preserves_its_public_sampling(reference_and_source, tmp_path):
    continuous = PathGeometry.from_continuous_reference(reference_and_source[0])
    geometry = PathGeometry.from_samples(continuous.positions_m, continuous.rotations,
                                         continuous.feedrate_m_s, continuous.source_time_s)
    values = {name: np.array(getattr(geometry, name), copy=True) for name in (
        "positions_m", "rotations", "quaternions_xyzw", "arc_length_m", "tangents",
        "omega_per_m", "feedrate_m_s", "source_time_s",
    )}
    saved = tmp_path / "legacy-path.npz"
    np.savez_compressed(saved, **values)
    with np.load(saved, allow_pickle=False) as archive:
        restored = PathGeometry.from_arrays({name: archive[name] for name in archive.files})
    assert restored.continuous_reference is None and restored.reference_identity == ""
    for progress in np.linspace(0., geometry.total_length_m, 11):
        np.testing.assert_array_equal(restored.sample(progress).rotation, geometry.sample(progress).rotation)
        np.testing.assert_array_equal(restored.sample(progress).position_m, geometry.sample(progress).position_m)


def test_c2_and_projection_degeneracy_are_rejected(reference_and_source):
    data = reference_and_source[0].arrays

    def rebuild(**changes):
        fields = dict(zip(("breaks_u", "coefficients_m", "projection_axes", "projection_centres_m",
                           "projection_radii_m", "source_u", "source_time_s", "source_feedrate_m_s"),
                          (data.breaks_u, data.coefficients_m, data.projection_axes, data.projection_centres_m,
                           data.projection_radii_m, data.source_u, data.source_time_s, data.source_feedrate_m_s)))
        fields.update(changes)
        return ContinuousPathReference.build(**fields)

    coefficients = data.coefficients_m.copy()
    coefficients[1, 3, 0] += 1e-3
    with pytest.raises(ValueError, match="C2"):
        rebuild(coefficients_m=coefficients)
    with pytest.raises(ValueError, match="projection"):
        rebuild(projection_radii_m=np.array([0., .2]))
    with pytest.raises(ValueError, match="有限"):
        rebuild(coefficients_m=np.full_like(data.coefficients_m, np.nan))
    with pytest.raises(ValueError, match="source"):
        rebuild(source_time_s=np.zeros_like(data.source_time_s))


@pytest.mark.parametrize("stationary_u", [0., .33, .123456789, .781234567, 1.])
@pytest.mark.parametrize("direction", ["axial", "angular"])
def test_stationary_points_inside_source_intervals_are_rejected(stationary_u, direction):
    coefficients = np.array([[[0., 1., 0.], [0., -3 * stationary_u, 0.],
                              [0., 3 * stationary_u**2, 0.], [1., -stationary_u**3, 0.]]])
    if direction == "angular":
        coefficients[:, :, [1, 2]] = coefficients[:, :, [2, 1]]
    with pytest.raises(ValueError, match="整个区间"):
        ContinuousPathReference.build(
            np.array([0., 1.]), coefficients, np.array([[0., 1., 0.], [0., 1., 0.]]),
            np.zeros((2, 3)), np.ones(2), np.array([0., 1.]), np.array([0., 1.]), np.zeros(2))


def test_positive_speed_between_source_samples_has_a_proven_lower_bound():
    u = .33
    coefficients = np.array([[[0., 1., 0.], [0., -3 * u, 0.],
                              [0., 3 * u**2 + .02, 0.], [1., -u**3, 0.]]])
    reference = ContinuousPathReference.build(
        np.array([0., 1.]), coefficients, np.array([[0., 1., 0.], [0., 1., 0.]]),
        np.zeros((2, 3)), np.ones(2), np.array([0., 1.]), np.array([0., 1.]), np.zeros(2))
    assert 1e-12 < reference.minimum_speed_m_per_u <= .02
