"""Shared joint-limit CBF clearance contract.

The values use metres for J1 and radians for J2--J9, matching the joint
coordinates.  They are controller safety semantics, not actuator limits.
"""

from __future__ import annotations


# A joint at exactly its mechanical endpoint has a negative JAX joint-limit
# barrier.  Keep this value shared by CBF construction and initial IK checks.
JOINT_LIMIT_CBF_MARGIN = 0.01

# Start one additional CBF margin inside the hard boundary.  This gives the
# first QP cycle positive room instead of beginning exactly on h(q) = 0.
START_IK_MIN_JOINT_MARGIN = 2.0 * JOINT_LIMIT_CBF_MARGIN
