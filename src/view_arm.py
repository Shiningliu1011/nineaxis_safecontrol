#!/usr/bin/env python3
"""
在 MuJoCo viewer 中查看 9 自由度机械臂 URDF
带地面环境，纯运动学模式（无重力）

坐标系约定:
  - URDF 使用 Y-up: Y 轴垂直地面向上
  - MuJoCo 使用 Z-up: Z 轴垂直地面向上
  - 通过 display_frame body 的 euler 旋转实现 Y-up → Z-up 转换

方法: MuJoCo 加载 URDF → 保存为 MJCF → 注入环境元素 → 重新加载
这样保证运动学完全正确 (由 MuJoCo 自己的 URDF 转换器处理)

用法: python3 view_arm.py
"""

import mujoco
import mujoco.viewer
import numpy as np
import os
import re
import scipy.io
import tempfile
from math import sqrt

# === 配置 ===
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF_PATH = os.path.join(PROJECT_ROOT, "models", "ninezzhou", "urdf", "ninezzhou.urdf")
MESH_DIR = os.path.join(PROJECT_ROOT, "models", "ninezzhou", "meshes")

# Y-up → Z-up 旋转: 绕 X 轴 +90°
Y_UP_TO_Z_UP_EULER = "1.5707963267948966 0 0"


def load_urdf_and_save_mjcf(urdf_path, mesh_dir):
    """用 MuJoCo 的 URDF 加载器加载, 然后保存为 MJCF"""
    with open(urdf_path, "r") as f:
        urdf_content = f.read()

    # 修复 mesh 路径
    urdf_fixed = urdf_content.replace(
        "package://ninezzhou/meshes/", mesh_dir + "/"
    )

    # 写入临时 URDF
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".urdf", delete=False
    ) as f:
        f.write(urdf_fixed)
        tmp_urdf = f.name

    try:
        # MuJoCo 加载 URDF (正确的运动学转换)
        model = mujoco.MjModel.from_xml_path(tmp_urdf)

        # 保存为 MJCF
        tmp_mjcf = tmp_urdf.replace(".urdf", "_converted.xml")
        mujoco.mj_saveLastXML(tmp_mjcf, model)

        # 读取 MJCF
        with open(tmp_mjcf) as f:
            mjcf_content = f.read()

        os.unlink(tmp_mjcf)
        return mjcf_content
    finally:
        os.unlink(tmp_urdf)


def inject_environment(mjcf_content):
    """向 MuJoCo 生成的 MJCF 中注入地面、灯光和坐标系旋转"""

    # 在 <worldbody> 开始后注入环境元素和 display_frame wrapper
    # 同时用 display_frame body 包裹整个机器人实现 Y-up → Z-up
    env_inject = f"""
    <!-- 地面 (Z-up 坐标系, XY 平面) -->
    <geom name="floor" type="plane" size="3 3 0.1" pos="0 0 -0.058" rgba="0.3 0.3 0.3 1"/>
    <!-- 灯光 -->
    <light pos="0 0 4" dir="0 0 -1" diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3"/>
    <light pos="2 2 3" dir="-1 -1 -1" diffuse="0.5 0.5 0.5"/>
    <light pos="-2 -2 3" dir="1 1 -1" diffuse="0.3 0.3 0.3"/>
    <!-- Y-up → Z-up 旋转容器 -->
    <body name="display_frame" pos="0 0 0" euler="{Y_UP_TO_Z_UP_EULER}">"""

    # 在 </worldbody> 前关闭 display_frame body
    close_frame = "    </body>"

    # 注入: 在 <worldbody> 后加环境 + 打开 display_frame
    result = mjcf_content.replace(
        "<worldbody>",
        "<worldbody>\n" + env_inject,
    )

    # 在 </worldbody> 前关闭 display_frame
    result = result.replace("</worldbody>", close_frame + "\n  </worldbody>")

    # 添加 gravity="0 0 0" 到 <option> 标签 (如果没有就插入)
    if "<option" in result:
        result = result.replace("<option", '<option gravity="0 0 0"', 1)
    else:
        result = result.replace(
            "</mujoco>",
            '  <option gravity="0 0 0"/>\n</mujoco>',
        )

    # 添加纹理和材质 asset
    texture_asset = """    <texture name="grid_tex" type="2d" builtin="checker" rgb1=".2 .2 .2" rgb2=".35 .35 .35" width="512" height="512"/>
    <material name="grid_mat" texture="grid_tex" texrepeat="4 4" reflectance="0.1"/>
"""
    result = result.replace(
        "</asset>",
        texture_asset + "  </asset>",
    )
    # 给地面加上材质
    result = result.replace(
        'rgba="0.3 0.3 0.3 1"',
        'material="grid_mat"',
    )

    return result


def load_trajectory(mat_path, max_points=300):
    """从 .mat 文件加载末端轨迹, 子采样, mm→m

    ``max_points=0`` 返回全部点 (用于圆柱拟合, 避免静止段使拟合退化)。
    """
    data = scipy.io.loadmat(mat_path)
    ik = data["ik_input"][0, 0]
    pos = ik["position_series"]  # (N, 3) 单位 mm
    n = len(pos)
    if max_points > 0 and n > max_points:
        idx = np.linspace(0, n - 1, max_points, dtype=int)
        pos = pos[idx]
    return pos / 1000.0  # mm → m


def inject_trajectory(mjcf_xml, points, offset=None, color="1 0 0 0.9", radius=0.001):
    """在 display_frame body 内注入轨迹线段 (capsule fromto)
    offset: [x, y, z] Y-up 坐标系下的平移偏移量 (米)
    """
    if offset is not None:
        points = points + np.array(offset)
    geoms = ""
    for i in range(len(points) - 1):
        p0 = points[i]
        p1 = points[i + 1]
        geoms += (
            f'      <geom name="traj_{i}" type="capsule" '
            f'fromto="{p0[0]:.6f} {p0[1]:.6f} {p0[2]:.6f} '
            f'{p1[0]:.6f} {p1[1]:.6f} {p1[2]:.6f}" '
            f'size="{radius}" rgba="{color}"/>\n'
        )
    # display_frame 的 </body> 是 </worldbody> 前最后一个 </body>
    idx = mjcf_xml.rfind("</body>")
    if idx == -1:
        return mjcf_xml
    return mjcf_xml[:idx] + geoms + mjcf_xml[idx:]


def inject_obstacles(mjcf_xml, obstacles):
    """在 display_frame body 内注入静态障碍物
    obstacles: list of (name, type, pos, size, rgba)
      type: "box" / "sphere" / "cylinder"
      pos: [x, y, z] Y-up 坐标 (米)
      size: box=[hx,hy,hz], sphere=[r], cylinder=[r,half_h]
    """
    geoms = ""
    for name, otype, pos, size, rgba in obstacles:
        pos_s = f"{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}"
        size_s = " ".join(f"{s:.6f}" for s in size)
        geoms += (
            f'      <geom name="{name}" type="{otype}" '
            f'pos="{pos_s}" size="{size_s}" '
            f'rgba="{rgba}"/>\n'
        )
    idx = mjcf_xml.rfind("</body>")
    if idx == -1:
        return mjcf_xml
    return mjcf_xml[:idx] + geoms + mjcf_xml[idx:]


# ---------------------------------------------------------------------------
#  Coordinate-frame axis visualisation
#  Red = X, Green = Y, Blue = Z.  Each frame is a child body with three
#  display-only capsule geoms (group 2).
# ---------------------------------------------------------------------------

FRAME_AXIS_LENGTH = 0.06   # m
FRAME_AXIS_RADIUS = 0.0025  # m
FRAME_AXIS_ALPHA = 0.85


def _frame_axis_geoms(frame_name, length=None, radius=None, alpha=None, raw=False):
    """Return MuJoCo XML for three axis capsule geoms.

    When *raw* is False the geoms are wrapped in a child ``body`` element
    with identity transform.  When *raw* is True only the ``geom`` lines
    are returned.
    """
    if length is None:
        length = FRAME_AXIS_LENGTH
    if radius is None:
        radius = FRAME_AXIS_RADIUS
    if alpha is None:
        alpha = FRAME_AXIS_ALPHA
    indent = "    " if raw else "          "
    geoms = (
        f'{indent}<geom name="{frame_name}_axis_x" type="capsule" '
        f'fromto="0 0 0 {length:.3f} 0 0" size="{radius:.4f}" '
        f'rgba="1 0.08 0.08 {alpha:.2f}" contype="0" conaffinity="0" group="2"/>\n'
        f'{indent}<geom name="{frame_name}_axis_y" type="capsule" '
        f'fromto="0 0 0 0 {length:.3f} 0" size="{radius:.4f}" '
        f'rgba="0.08 1 0.08 {alpha:.2f}" contype="0" conaffinity="0" group="2"/>\n'
        f'{indent}<geom name="{frame_name}_axis_z" type="capsule" '
        f'fromto="0 0 0 0 0 {length:.3f}" size="{radius:.4f}" '
        f'rgba="0.08 0.25 1 {alpha:.2f}" contype="0" conaffinity="0" group="2"/>\n'
    )
    if raw:
        return geoms
    return (
        f'        <body name="{frame_name}_frame_axes" pos="0 0 0" quat="1 0 0 0">\n'
        f'{geoms}'
        f'        </body>\n'
    )


def inject_coord_frame_axes(mjcf_xml):
    """Add axis geoms for the base (world origin) and tool0 (Link9+offset)
    coordinate frames."""

    # Base frame: insert right after the display_frame opening tag.
    base_viz = _frame_axis_geoms(
        "base_link",
        FRAME_AXIS_LENGTH * 1.5,
        FRAME_AXIS_RADIUS * 1.2,
    )
    display_pattern = re.compile(
        r'(<body\s[^>]*\bname="display_frame"[^>]*>)'
    )
    xml = display_pattern.sub(
        lambda m: m.group(0) + "\n" + base_viz, mjcf_xml, count=1
    )

    # Tool0 frame: insert as a child of Link9 at position (0.235, 0, 0).
    # This matches the URDF tool0_fixed joint origin xyz="0.235 0 0".
    tool0_viz = (
        f'        <body name="tool0_frame_axes" pos="0.235 0 0" quat="1 0 0 0">\n'
    )
    tool0_viz += _frame_axis_geoms(
        "tool0", FRAME_AXIS_LENGTH, FRAME_AXIS_RADIUS, raw=True
    )
    tool0_viz += "        </body>\n"
    link9_pattern = re.compile(
        r'(<body\s[^>]*\bname="Link9"[^>]*>)'
    )
    xml = link9_pattern.sub(
        lambda m: m.group(0) + "\n" + tool0_viz, xml, count=1
    )

    return xml


# ---------------------------------------------------------------------------
#  Cylinder fitting & surface-normal visualisation
# ---------------------------------------------------------------------------

def fit_cylinder(points, axis_direction, height_margin_m=0.04):
    """Fit a circle in the plane perpendicular to *axis_direction*.

    Returns ``(center, axis, radius, height)`` where all values are in the
    same (Y-up) coordinate frame as the input points.
    """
    if len(points) < 3:
        raise ValueError("Need at least 3 points to fit a cylinder")
    values = np.asarray(points, dtype=float)
    axis = np.asarray(axis_direction, dtype=float)
    axis_len = float(np.linalg.norm(axis))
    if axis_len < 1e-12:
        raise ValueError("zero-length axis_direction")
    axis /= axis_len

    # Orthonormal basis perpendicular to axis
    helper = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(helper, axis))) > 0.9:
        helper = np.array([0.0, 0.0, 1.0])
    u = np.cross(axis, helper)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    v /= np.linalg.norm(v)

    plane_x = values @ u
    plane_y = values @ v
    A = np.column_stack((plane_x, plane_y, np.ones(len(values))))
    target = -(plane_x * plane_x + plane_y * plane_y)
    coeff, *_ = np.linalg.lstsq(A, target, rcond=None)
    d_val, e_val, f_val = coeff
    cx = -0.5 * d_val
    cy = -0.5 * e_val
    r_sq = cx * cx + cy * cy - f_val
    if r_sq <= 0.0:
        raise ValueError("Cylinder fit produced non-positive radius")
    radius = sqrt(float(r_sq))

    axial_vals = values @ axis
    axial_centre = 0.5 * (float(axial_vals.min()) + float(axial_vals.max()))
    height = float(axial_vals.max() - axial_vals.min()) + 2.0 * height_margin_m
    center = cx * u + cy * v + axial_centre * axis

    # Error stats
    radial_dist = np.sqrt((plane_x - cx) ** 2 + (plane_y - cy) ** 2)
    radial_err = radial_dist - radius
    rms_err = float(np.sqrt(np.mean(radial_err * radial_err)))
    max_err = float(np.max(np.abs(radial_err)))

    return dict(
        center=tuple(float(v) for v in center),
        axis=tuple(float(v) for v in axis),
        radius=radius,
        height=height,
        rms_error_mm=rms_err * 1000.0,
        max_error_mm=max_err * 1000.0,
    )


def inject_cylinder_normals(mjcf_xml, points, cylinder, normal_length=0.04, step=15):
    """Add capsule geoms pointing *toward* the cylinder axis at sampled points.

    The direction matches the tool0 X-axis orientation used by IK (inward
    radial), since the outward normal is below the robot's reachable workspace.
    """
    center = np.asarray(cylinder["center"], dtype=float)
    axis = np.asarray(cylinder["axis"], dtype=float)
    geoms = ""
    for idx in range(0, len(points), step):
        p = np.asarray(points[idx], dtype=float)
        rel = p - center
        axial = axis * float(np.dot(rel, axis))
        radial = rel - axial
        rlen = float(np.linalg.norm(radial))
        if rlen < 1e-12:
            continue
        inward = -radial / rlen
        end = p + inward * normal_length
        geoms += (
            f'      <geom name="surface_normal_{idx}" type="capsule" '
            f'fromto="{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} '
            f'{end[0]:.6f} {end[1]:.6f} {end[2]:.6f}" '
            f'size="0.0018" rgba="0.15 0.85 0.15 0.85" '
            f'contype="0" conaffinity="0" group="2"/>\n'
        )
    if not geoms:
        return mjcf_xml
    # Insert normals before the last </body> that precedes </worldbody>.
    # This is the same insertion strategy used by inject_trajectory and
    # inject_obstacles, ensuring normals stay inside the display_frame body.
    wb_pos = mjcf_xml.rfind("</worldbody>")
    body_pos = mjcf_xml.rfind("</body>", 0, wb_pos)
    if body_pos == -1:
        return mjcf_xml
    return mjcf_xml[:body_pos] + geoms + mjcf_xml[body_pos:]


def main():
    # Step 1: 用 MuJoCo 加载 URDF 并保存为 MJCF
    mjcf_raw = load_urdf_and_save_mjcf(URDF_PATH, MESH_DIR)

    # Step 2: 注入环境元素
    mjcf_xml = inject_environment(mjcf_raw)

    # Step 2.5: 注入末端工作曲线
    traj_pts_offset = []
    traj_pts_full_offset = []
    traj_path = os.path.join(PROJECT_ROOT, "data", "nurbs", "ik_input.mat")
    traj_offset = np.array([0.0, 0.343, 1.587])
    if os.path.exists(traj_path):
        traj_pts_raw = load_trajectory(traj_path, max_points=300)
        traj_pts_offset = traj_pts_raw + traj_offset
        mjcf_xml = inject_trajectory(mjcf_xml, traj_pts_raw, offset=[0.0, 0.343, 1.587])
        # 全部点用于圆柱拟合 (max_points=0), 避免静止段使圆拟合退化
        traj_pts_full_offset = load_trajectory(traj_path, max_points=0) + traj_offset

    # Step 2.6: 注入静态障碍物 (Y-up 坐标)
    # 各 Link 位置: Link3(0,0.343,0.225) Link5(0,0.343,0.793) Link7(0,0.343,0.928)
    obstacles = [
        # (name, type, pos, half_size, rgba)
        ("obs_box1",    "box",      [0.25, 0.243, 0.4],   [0.04, 0.04, 0.08], "0.2 0.7 0.2 0.8"),
        ("obs_sphere1", "sphere",   [-0.25, 0.343, 0.6],  [0.05],              "0.9 0.6 0.1 0.8"),
        ("obs_cyl1",    "cylinder", [0.22, 0.30, 0.9],    [0.03, 0.08],        "0.2 0.5 0.9 0.8"),
        ("obs_box2",    "box",      [-0.1, 0.15, 0.9],   [0.05, 0.05, 0.05],  "0.8 0.2 0.2 0.8"),
    ]
    mjcf_xml = inject_obstacles(mjcf_xml, obstacles)

    # Step 2.7: 注入基座 (base_link) 和末端 (tool0) 坐标系轴
    mjcf_xml = inject_coord_frame_axes(mjcf_xml)

    # Step 2.8: 圆柱面拟合 & 法线方向可视化
    # 用全部轨迹点拟合圆柱 (静止段使圆拟合退化, 法线方向变成数值噪声)
    if len(traj_pts_full_offset) > 0:
        try:
            cyl = fit_cylinder(traj_pts_full_offset, [0.0, 1.0, 0.0])
            print(
                f"圆柱面拟合(全部点): center={cyl['center']}, "
                f"radius={cyl['radius']:.4f}m, "
                f"height={cyl['height']:.4f}m, "
                f"RMS误差={cyl['rms_error_mm']:.3f}mm, "
                f"最大误差={cyl['max_error_mm']:.3f}mm"
            )
            # 法线画在显示轨迹点处, 方向基于正确拟合的圆柱
            mjcf_xml = inject_cylinder_normals(mjcf_xml, traj_pts_offset, cyl)
        except ValueError as e:
            print(f"警告: 圆柱面拟合失败 - {e}")

    # Step 3: 保存最终 MJCF
    mjcf_path = os.path.join(PROJECT_ROOT, "output", "ninezzhou_env.xml")
    with open(mjcf_path, "w") as f:
        f.write(mjcf_xml)
    print(f"MJCF 已保存: {mjcf_path}")

    # Step 4: 加载带环境的模型
    model = mujoco.MjModel.from_xml_string(mjcf_xml)
    data = mujoco.MjData(model)

    # 禁用物理仿真: timestep=0 让 mj_step 不产生任何位移
    model.opt.timestep = 0

    # 验证
    mujoco.mj_forward(model, data)
    print(f"\n模型加载成功!")
    print(f"  关节数量: {model.njnt}")
    print(f"  自由度:   {model.nv}")
    print(f"  刚体数量: {model.nbody}")
    print(f"  重力:     {tuple(model.opt.gravity)} (已禁用)")
    print(f"  坐标系:   Y-up → Z-up (display_frame euler={Y_UP_TO_Z_UP_EULER})")
    print(f"  显示元素: 红色=X轴 绿色=Y轴 蓝色=Z轴 (base_link / tool0)")
    print(f"  显示元素: 绿色短线段 = 圆柱面法线方向")
    print(f"\n关节列表:")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        jnt_range = model.jnt_range[i]
        type_str = (
            "prismatic"
            if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_SLIDE
            else "revolute"
        )
        unit = "m" if type_str == "prismatic" else "rad"
        print(
            f"  J{i+1} ({name}): {type_str:10s}  "
            f"[{jnt_range[0]:+.4f}, {jnt_range[1]:+.4f}] {unit}"
        )

    # 打印 body 位置 (Z-up 坐标)
    print(f"\nBody 位置 (Z-up):")
    for i in range(model.nbody):
        bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        p = data.xpos[i]
        print(f"  {bname:<15} ({p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f})")

    print(f"\n启动 MuJoCo viewer...")
    print(f"  鼠标左键拖动: 旋转视角 | 右键拖动: 平移 | 滚轮: 缩放")
    print(f"  右侧面板可拖动滑块控制各关节 | 按 'q' 退出\n")

    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
