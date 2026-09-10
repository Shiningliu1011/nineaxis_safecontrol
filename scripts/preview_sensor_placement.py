#!/usr/bin/env python3
"""Standalone MuJoCo placement study; no ROS node or hardware commands.

Run after sourcing /opt/ros/humble/setup.bash. Config uses base_link Y-up.
C/L toggle camera/LiDAR FOV; W workspace; B sampled link envelope. Joint sliders pose the arm without dynamics.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from robot_safecontrol_moveit.mujoco_viewer_with_cylinder import MuJoCoJointStateViewer as ExistingViewer
from robot_safecontrol_moveit.robot_spec import DEFAULT_JOINT_NAMES
from sensor_placement_geometry import coverage, statistics, lidar_rotation


def numbers(values):
    return ' '.join(f'{float(x):.7g}' for x in values)


def line(parent, a, b, color, group=0, radius=0.003):
    ET.SubElement(parent, 'geom', type='capsule', fromto=numbers([*a, *b]),
                  size=str(radius), rgba=numbers(color), group=str(group),
                  contype='0', conaffinity='0')


def shared_stand(frame, config):
    """Conceptual fixed stand in base_link; not a fabrication/structural model."""
    stand = config['shared_stand']
    x, z = stand['mast_x'], stand['mast_z']
    height, half_width = stand['beam_height_m'], stand['beam_half_width_m']
    body = ET.SubElement(frame, 'body', name='shared_sensor_stand')
    metal = [0.65, 0.7, 0.75, 1]
    ET.SubElement(body, 'geom', name='stand_base', type='box',
                  pos=numbers([x, -0.035, z]), size='0.36 0.02 0.24',
                  rgba='0.3 0.35 0.4 1', contype='0', conaffinity='0')
    line(body, [x, -0.015, z], [x, height, z], metal, radius=0.025)
    line(body, [x-half_width, height, z], [x+half_width, height, z], metal, radius=0.02)
    for sensor in ['camera', 'lidar']:
        point = np.asarray(config[sensor]['position'])
        if sensor == 'lidar':
            rotation = lidar_rotation(config[sensor])
            foot = point - 0.05*rotation[:, 1]
            quat = np.empty(4)
            mujoco.mju_mat2Quat(quat, rotation.ravel())
            ET.SubElement(body, 'geom', name='lidar_mount_plate', type='box',
                          pos=numbers(point-0.035*rotation[:, 1]), quat=numbers(quat),
                          size='0.055 0.003 0.05', rgba=numbers(metal),
                          contype='0', conaffinity='0')
        else:
            foot = point + [0, -0.03, -0.025]
        line(body, [point[0], height, z], foot, metal, radius=0.012)
        line(body, foot, point, metal, radius=0.008)
    return body


def scene(config):
    xml = ExistingViewer._urdf_to_mjcf(ROOT / 'models/ninezzhou/urdf/ninezzhou.urdf', ROOT / 'models/ninezzhou/meshes')
    xml = ExistingViewer._inject_display_scene(xml, [], joint_names=DEFAULT_JOINT_NAMES)
    root = ET.fromstring(xml)
    frame = ET.SubElement(root.find('worldbody'), 'body', name='sensor_preview_base_link', euler='1.5707963267948966 0 0')
    if config.get('shared_stand', {}).get('enabled', False):
        shared_stand(frame, config)
    cam = config['camera']
    p = np.array(cam['position'])
    forward = np.array(cam['look_at']) - p
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0, 1, 0]); right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, np.column_stack([right, up, -forward]).ravel())
    body = ET.SubElement(frame, 'body', name='Gemini_335L_candidate', pos=numbers(p), quat=numbers(quat))
    ET.SubElement(body, 'geom', type='box', size='0.062 0.0145 0.0135', rgba='0.1 0.75 1 1', contype='0', conaffinity='0')
    for x in [-0.0475, 0.0475]:
        ET.SubElement(body, 'geom', type='sphere', pos=f'{x} 0 -0.014', size='0.008', rgba='0.02 0.05 0.08 1', contype='0', conaffinity='0')
    if not config.get('shared_stand', {}).get('enabled', False):
        line(frame, [p[0], -0.05, p[2]], p, [0.3, 0.4, 0.45, 1], radius=0.014)
    rings = []
    for distance in [cam['near_m'], cam['display_far_m']]:
        w = distance * np.tan(np.deg2rad(cam['horizontal_fov_deg']/2))
        h = distance * np.tan(np.deg2rad(cam['vertical_fov_deg']/2))
        ring = [p + distance*forward + sx*w*right + sy*h*up for sx, sy in [(-1,-1), (1,-1), (1,1), (-1,1)]]
        rings.append(ring)
        for i in range(4):
            line(frame, ring[i], ring[(i+1)%4], [0.1,0.8,1,0.65], 3)
    for a,b in zip(*rings):
        line(frame, a,b, [0.1,0.8,1,0.5], 3)
    line(frame, p, cam['look_at'], [0.1,0.8,1,0.6], 3, 0.002)
    lidar = config['lidar']; p = np.array(lidar['position'])
    rotation = lidar_rotation(lidar)
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, rotation.ravel())
    body = ET.SubElement(frame, 'body', name='Mid360S_candidate', pos=numbers(p), quat=numbers(quat))
    ET.SubElement(body, 'geom', type='box', size='0.0325 0.03 0.0325', rgba='1 0.55 0.1 1', contype='0', conaffinity='0')
    if not config.get('shared_stand', {}).get('enabled', False):
        line(frame, [p[0],-0.05,p[2]], p, [0.4,0.35,0.25,1], radius=0.014)
    radius = lidar['display_radius_m']
    for degrees in [lidar['elevation_min_deg'], 0, lidar['elevation_max_deg']]:
        elev = np.deg2rad(degrees)
        ring = [p + radius*(rotation @ np.array([np.cos(elev)*np.cos(a), np.sin(elev), np.cos(elev)*np.sin(a)])) for a in np.linspace(0,2*np.pi,73)]
        for a,b in zip(ring,ring[1:]):
            line(frame,a,b,[1,0.55,0.12,0.45],4,0.002)
        for a in ring[:-1:6]:
            line(frame,p,a,[1,0.55,0.12,0.25],4,0.0015)
    return ET.tostring(root, encoding='unicode')


def add_workspace(xml, config, samples=8192):
    """Kinematic samples, NOT collision-free or orientation-constrained workspace.

    Tool0 points are colored by nominal sensor FOV membership.
    Purple hull is an outer envelope of
    sampled link mesh surface points; its interior is not certified reachable.
    All points here already use the MuJoCo world frame, so no second rotation.
    """
    from scipy.spatial import ConvexHull
    from scipy.stats import qmc
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    joints = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
              for n in DEFAULT_JOINT_NAMES]
    if any(j < 0 or not model.jnt_limited[j] for j in joints):
        raise ValueError('Workspace requires finite limits for every robot joint')
    tool = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'tool0_frame_axes')
    if tool < 0:
        raise ValueError('Missing tool0 frame')
    limits = model.jnt_range[joints]
    unit = qmc.Sobol(len(joints), scramble=True, seed=42).random_base2(
        int(np.log2(samples)))
    poses = limits[:, 0] + unit * (limits[:, 1] - limits[:, 0])
    poses = np.vstack([poses, np.zeros(len(joints)), limits[:, 0], limits[:, 1]])
    # Keep directional extrema of each mesh, preserving actual mesh transforms.
    meshes = []
    for g in range(model.ngeom):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                    model.geom_bodyid[g]) or ''
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or not body_name.startswith('Link'):
            continue
        mesh = model.geom_dataid[g]
        start = model.mesh_vertadr[mesh]
        vertices = model.mesh_vert[start:start + model.mesh_vertnum[mesh]]
        indices = np.unique(np.r_[vertices.argmin(axis=0), vertices.argmax(axis=0)])
        meshes.append((g, vertices[indices]))
    tips, surfaces = [], []
    for pose in poses:
        data.qpos[model.jnt_qposadr[joints]] = pose
        mujoco.mj_kinematics(model, data)
        tips.append(data.xpos[tool].copy())
        surfaces.extend(vertices @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
                        for g, vertices in meshes)
    tips = np.asarray(tips)
    base_tips = tips @ np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    (ROOT / 'output').mkdir(exist_ok=True)
    np.save(ROOT / 'output/workspace_base_samples.npy', base_tips)
    # Voxel representatives keep the original reachable sample, not a voxel centre.
    _, indices = np.unique(np.floor(tips / 0.055).astype(int), axis=0, return_index=True)
    shown = tips[indices]
    if len(shown) > 3000:
        shown = shown[np.linspace(0, len(shown)-1, 3000).astype(int)]
    root = ET.fromstring(xml)
    world = root.find('worldbody')
    shown_base = shown @ np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    camera_visible, lidar_visible = coverage(shown_base, config)
    colors = {(True, True): '0.25 1 0.45 0.5',
              (True, False): '0.1 0.8 1 0.85',
              (False, True): '1 0.55 0.12 0.85',
              (False, False): '1 0.05 0.1 1'}
    for point, cam_ok, lidar_ok in zip(shown, camera_visible, lidar_visible):
        ET.SubElement(world, 'geom', type='sphere', pos=numbers(point), size='0.009',
                      rgba=colors[(bool(cam_ok), bool(lidar_ok))], group='5', contype='0', conaffinity='0')
    print('Nominal FOV sample counts:', statistics(base_tips, config), flush=True)
    hull = ConvexHull(np.concatenate(surfaces))
    edges = {tuple(sorted((int(a), int(b)))) for face in hull.simplices
             for a, b in zip(face, np.roll(face, -1))}
    for a, b in sorted(edges):
        line(world, hull.points[a], hull.points[b], [0.8, 0.45, 1, 0.32], 2, 0.002)
    # Use the true joint ranges for the preview's pose controls.
    for actuator, bounds in zip(root.find('actuator'), limits):
        actuator.set('ctrllimited', 'true')
        actuator.set('ctrlrange', numbers(bounds))
    base_tips = tips @ np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    print(f'Workspace: {len(poses)} poses, {len(shown)} displayed tool samples; '
          f'base_link sample bounds {base_tips.min(axis=0)} .. {base_tips.max(axis=0)}. '
          'No self/environment collision rejection; no tool orientation constraint.', flush=True)
    return ET.tostring(root, encoding='unicode')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/sensor_placement_preview.json')
    parser.add_argument('--render', type=Path, help='Save preview PNG instead of opening window')
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    model = mujoco.MjModel.from_xml_string(add_workspace(scene(config), config))
    model.opt.gravity[:] = 0
    data = mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = config.get('viewer', {}).get('lookat_world', [0, -0.8, 0.5])
    camera.distance = config.get('viewer', {}).get('distance', 7.6)
    camera.azimuth = config.get('viewer', {}).get('azimuth', 135)
    camera.elevation = config.get('viewer', {}).get('elevation', -25)
    if args.render:
        from PIL import Image
        model.vis.global_.offwidth = 1400; model.vis.global_.offheight = 1000
        with mujoco.Renderer(model, height=1000, width=1400) as renderer:
            option = mujoco.MjvOption()
            option.geomgroup[3:6] = 1
            option.geomgroup[2] = 0
            renderer.update_scene(data,camera=camera,scene_option=option)
            renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
            args.render.parent.mkdir(parents=True,exist_ok=True)
            Image.fromarray(renderer.render()).save(args.render)
        print(args.render)
        return
    pending = []
    with mujoco.viewer.launch_passive(model,data,key_callback=lambda key: pending.append(key)) as viewer:
        viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        viewer.opt.geomgroup[3:6] = 1
        viewer.opt.geomgroup[2] = 0
        viewer.cam.lookat[:] = camera.lookat
        viewer.cam.distance = camera.distance; viewer.cam.azimuth = camera.azimuth; viewer.cam.elevation = camera.elevation
        print('Candidate preview: cyan=Gemini FOV; orange=Mid360S FOV. C/L toggle sensors; W workspace (green=both, cyan=camera, orange=lidar, red=neither); B purple link envelope. Joint sliders pose arm.', flush=True)
        while viewer.is_running():
            with viewer.lock():
                while pending:
                    key = pending.pop(0)
                    if key in (67, 76, 87, 66):
                        group = {67: 3, 76: 4, 87: 5, 66: 2}[key]
                        viewer.opt.geomgroup[group] = 1-viewer.opt.geomgroup[group]
                for joint in range(model.njnt):
                    name = mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_JOINT,joint)
                    actuator = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,f'act_{name}')
                    if actuator >= 0:
                        data.qpos[model.jnt_qposadr[joint]] = data.ctrl[actuator]
                mujoco.mj_forward(model,data)
            viewer.sync(); time.sleep(1/30)


if __name__ == '__main__':
    main()
