# 由 portable_oscbf/scripts/generate_kinematics_data.py 生成。
# 手工来源：models/ninezzhou/urdf/ninezzhou.urdf。

from work.robot_geometry import LINK_NAMES


JOINT_CHAIN = [
    ('world', 'base_link', 'fixed', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 1.0)),
    ('base_link', 'Link1', 'prismatic', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 1.0)),
    ('Link1', 'Link2', 'revolute', 0.0, 0.343, 0.0, 1.5708, -1.5708, 0.0, (0.0, 0.0, 1.0)),
    ('Link2', 'Link3', 'revolute', 0.225, 0.0, 0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 1.0)),
    ('Link3', 'Link4', 'revolute', 0.225, 0.0, 0.0, 0.0, 0.0, 1.5708, (0.0, 0.0, 1.0)),
    ('Link4', 'Link5', 'revolute', 0.0, -0.343, 0.0, -1.5708, 0.0, -3.1416, (0.0, 0.0, 1.0)),
    ('Link5', 'Link6', 'revolute', 0.0, 0.0, 0.0, 1.5708, -1.5708, 0.0, (0.0, 0.0, 1.0)),
    ('Link6', 'Link7', 'revolute', 0.135, 0.0, 0.0, -1.5708, 0.0, 0.0, (0.0, 0.0, 1.0)),
    ('Link7', 'Link8', 'revolute', 0.11, 0.0, 0.0, 1.5708, 0.0, 0.0, (0.0, 0.0, 1.0)),
    ('Link8', 'Link9', 'revolute', 0.114, 0.0, 0.0, -1.5708, 0.0, 0.0, (0.0, 0.0, 1.0)),
    ('Link9', 'ee_link', 'fixed', 0.235, 0.0, 0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 1.0)),
]

JOINT_POSITION_LOWER = (
    0.0,
    -1.5708,
    -1.5708,
    -1.5708,
    -3.1416,
    -1.48353,
    -1.48353,
    -1.48353,
    -1.48353,
)

JOINT_POSITION_UPPER = (
    0.585,
    1.5708,
    1.5708,
    1.5708,
    3.1416,
    1.48353,
    1.48353,
    1.48353,
    1.48353,
)

N_JOINTS = 9
