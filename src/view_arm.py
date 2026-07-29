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
import scipy.io
import tempfile

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
    """从 .mat 文件加载末端轨迹, 子采样, mm→m"""
    data = scipy.io.loadmat(mat_path)
    ik = data["ik_input"][0, 0]
    pos = ik["position_series"]  # (N, 3) 单位 mm
    n = len(pos)
    if n > max_points:
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


def main():
    # Step 1: 用 MuJoCo 加载 URDF 并保存为 MJCF
    mjcf_raw = load_urdf_and_save_mjcf(URDF_PATH, MESH_DIR)

    # Step 2: 注入环境元素
    mjcf_xml = inject_environment(mjcf_raw)

    # Step 2.5: 注入末端工作曲线
    traj_path = os.path.join(PROJECT_ROOT, "data", "nurbs", "ik_input.mat")
    if os.path.exists(traj_path):
        traj_pts = load_trajectory(traj_path, max_points=300)
        mjcf_xml = inject_trajectory(mjcf_xml, traj_pts, offset=[0.0, 0.343, 1.587])

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

    # Step 3: 保存最终 MJCF

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
