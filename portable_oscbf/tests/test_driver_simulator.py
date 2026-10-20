import numpy as np
import pytest

from work.driver_simulator import SCurveDriverSimulator


def make_driver(dt=0.01):
    driver = SCurveDriverSimulator(
        np.array([0.5, 2.0]), np.array([5.0, 10.0]),
        q_min=np.array([0.0, -2.0]), q_max=np.array([0.585, 2.0]),
        jerk_time=0.08, dt=dt,
    )
    driver.reset(np.array([0.1, 0.0]))
    return driver


def test_moving_targets_and_final_hold_respect_continuous_limits():
    driver = make_driver()
    previous_velocity = driver.velocity
    previous_acceleration = driver.acceleration
    target = np.array([0.1, 0.0])
    for step in range(600):
        # 目标以不同于执行器的频率更新，并包含方向改变和最终保持。
        if step < 300 and step % 3 == 0:
            target = np.array([0.2 + 0.05 * np.sin(step * 0.01),
                               0.3 * np.sin(step * 0.02)])
        position = driver.step(target)
        velocity, acceleration = driver.velocity, driver.acceleration
        assert np.all(np.abs(velocity) <= np.array([0.5, 2.0]) + 1e-10)
        assert np.all(np.abs(acceleration) <= np.array([5.0, 10.0]) + 1e-10)
        assert np.all(np.abs(velocity - previous_velocity) <= np.array([5.0, 10.0]) * 0.01 + 1e-10)
        assert np.all(np.abs(acceleration - previous_acceleration) <= np.array([5.0, 10.0]) / 0.08 * 0.01 + 1e-10)
        assert np.all(position >= np.array([0.0, -2.0]) - 1e-10)
        assert np.all(position <= np.array([0.585, 2.0]) + 1e-10)
        previous_velocity, previous_acceleration = velocity, acceleration
    np.testing.assert_allclose(position, target, atol=1e-9)
    np.testing.assert_allclose(velocity, 0.0, atol=1e-9)
    np.testing.assert_allclose(acceleration, 0.0, atol=1e-9)


def test_stationary_boundary_and_reset():
    driver = make_driver()
    target = np.array([0.0, -2.0])
    for _ in range(400):
        position = driver.step(target)
    np.testing.assert_allclose(position, target, atol=1e-9)
    driver.reset(np.array([0.585, 2.0]))
    np.testing.assert_allclose(driver.step(np.array([0.585, 2.0])), [0.585, 2.0])
    np.testing.assert_array_equal(driver.velocity, np.zeros(2))
    np.testing.assert_array_equal(driver.acceleration, np.zeros(2))


@pytest.mark.parametrize("dt", [0.01, 0.002])
def test_constant_speed_target_has_no_repeated_acceleration(dt):
    driver = make_driver(dt)
    velocities, accelerations = [], []
    for step in range(round(6.0 / dt)):
        if step % round(0.02 / dt) == 0:
            target = np.array([0.1 + 0.02 * step * dt, 0.08 * step * dt])
        driver.step(target)
        if step >= round(2.0 / dt):
            velocities.append(driver.velocity)
            accelerations.append(driver.acceleration)
    np.testing.assert_allclose(velocities, np.broadcast_to([0.02, 0.08], (len(velocities), 2)), atol=1e-6)
    np.testing.assert_allclose(accelerations, 0.0, atol=1e-6)


@pytest.mark.parametrize("target", [[np.nan, 0.0], [0.1], [-0.01, 0.0]])
def test_invalid_target_is_rejected(target):
    with pytest.raises(ValueError):
        make_driver().step(np.array(target))
