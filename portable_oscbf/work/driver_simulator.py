"""S-curve driver simulator — jerk-limited velocity transition smoother.

Smooths velocity commands **only during state transitions** (phase switches).
During steady-state tracking, QP output passes through unfiltered to preserve
precision.

Usage::

    driver = SCurveDriverSimulator(dq_max, ddq_max)

    # Steady-state: pass through
    v_out = driver.step(u_safe)           # u_safe unchanged

    # State transition: smooth ramp
    driver.begin_transition()              # mark transition start
    for _ in range(N):
        v_out = driver.step(u_target)      # S-curve filtered

Algorithm (per-joint, per-step, during transition only)::

    Δv = v_target - v_current
    a_cmd = clip(kp * Δv, -a_max, a_max)
    da = clip(a_cmd - a, -j_max * dt, j_max * dt)
    a = clip(a + da, -a_max, a_max)
    v = clip(v + a * dt, -v_max, v_max)
"""

from __future__ import annotations

import numpy as np


class SCurveDriverSimulator:
    """Per-joint S-curve velocity transition smoother.

    Parameters
    ----------
    dq_max : array-like (9,)
        Joint velocity limits (m/s for J1, rad/s for J2-J9).
    ddq_max : array-like (9,)
        Joint acceleration limits.
    jerk_time : float
        Time constant for jerk limit: j_max = ddq_max / jerk_time.
    kp : float
        Velocity-loop proportional gain during transitions.
    dt : float
        Control period (s).
    transition_steps : int
        Number of steps to smooth a transition.  Default 25 (50ms @500Hz).
    """

    def __init__(
        self,
        dq_max: np.ndarray,
        ddq_max: np.ndarray,
        jerk_time: float = 0.08,
        kp: float = 80.0,
        dt: float = 0.002,
        transition_steps: int = 25,
    ) -> None:
        self._v_max = np.asarray(dq_max, dtype=float).ravel()
        self._a_max = np.asarray(ddq_max, dtype=float).ravel()
        self._j_max = self._a_max / max(float(jerk_time), 1e-4)
        self._kp = float(kp)
        self._dt = float(dt)
        self._n = len(self._v_max)
        self._transition_steps = int(transition_steps)

        # State
        self._v = np.zeros(self._n)
        self._a = np.zeros(self._n)
        self._in_transition = False
        self._transition_counter = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def begin_transition(self) -> None:
        """Mark the start of a velocity transition (e.g. state switch).

        After calling this, the next ``transition_steps`` calls to ``step()``
        will apply S-curve filtering.  After that, passthrough resumes.
        """
        self._in_transition = True
        self._transition_counter = 0

    def step(self, v_target: np.ndarray) -> np.ndarray:
        """Advance one control step.

        During transition: returns S-curve smoothed velocity.
        During steady-state: returns v_target unchanged (passthrough).

        Parameters
        ----------
        v_target : np.ndarray (n,)
            Target velocity from QP.

        Returns
        -------
        v_out : np.ndarray (n,)
            Output velocity (smoothed during transition, passthrough otherwise).
        """
        target = np.asarray(v_target, dtype=float).ravel()

        if not self._in_transition:
            # Steady-state: passthrough, but track velocity for next transition
            self._v = target.copy()
            self._a[:] = 0.0
            return target

        # Transition: S-curve filtering
        dt = self._dt
        kp = self._kp

        dv = target - self._v
        a_cmd = np.clip(kp * dv, -self._a_max, self._a_max)

        da = np.clip(a_cmd - self._a, -self._j_max * dt, self._j_max * dt)
        self._a = np.clip(self._a + da, -self._a_max, self._a_max)
        self._v = np.clip(self._v + self._a * dt, -self._v_max, self._v_max)

        self._transition_counter += 1
        if self._transition_counter >= self._transition_steps:
            self._in_transition = False

        return self._v.copy()

    def reset(self, v0: np.ndarray | None = None) -> None:
        """Reset driver state."""
        if v0 is not None:
            self._v = np.asarray(v0, dtype=float).ravel().copy()
        else:
            self._v = np.zeros(self._n)
        self._a = np.zeros(self._n)
        self._in_transition = False
        self._transition_counter = 0

    @property
    def in_transition(self) -> bool:
        """True if currently in a transition."""
        return self._in_transition

    @property
    def velocity(self) -> np.ndarray:
        """Current smoothed velocity."""
        return self._v.copy()

    @property
    def acceleration(self) -> np.ndarray:
        """Current acceleration."""
        return self._a.copy()
