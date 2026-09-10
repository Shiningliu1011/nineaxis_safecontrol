"""Nominal geometric FOV checks in base_link; excludes occlusion/detection physics."""
import numpy as np


def camera_basis(camera):
    forward = np.asarray(camera['look_at'], dtype=float) - camera['position']
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0, 1, 0])
    right /= np.linalg.norm(right)
    return right, np.cross(right, forward), forward


def lidar_rotation(lidar):
    """Local Y-up display axes to base_link; +pitch tilts local up toward +Z."""
    angle = np.deg2rad(lidar.get('tilt_toward_positive_z_deg', 0.0))
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def coverage(points, config, display_only=False):
    camera, lidar = config['camera'], config['lidar']
    relative = np.asarray(points) - camera['position']
    right, up, forward = camera_basis(camera)
    depth = relative @ forward
    camera_mask = ((depth >= camera['near_m']) &
                   (depth <= camera['display_far_m'] if display_only else depth <= camera.get('evaluation_far_m', 6.0)) &
                   (np.abs(relative @ right) <= depth*np.tan(np.deg2rad(camera['horizontal_fov_deg']/2))) &
                   (np.abs(relative @ up) <= depth*np.tan(np.deg2rad(camera['vertical_fov_deg']/2))))
    relative = (np.asarray(points) - lidar['position']) @ lidar_rotation(lidar)
    distance = np.linalg.norm(relative, axis=1)
    elevation = np.rad2deg(np.arctan2(relative[:, 1], np.linalg.norm(relative[:, [0, 2]], axis=1)))
    lidar_mask = ((distance >= lidar.get('evaluation_near_m', 0.2)) &
                  (distance <= lidar['display_radius_m'] if display_only else distance <= lidar.get('evaluation_far_m', 40.0)) &
                  (elevation >= lidar['elevation_min_deg']) &
                  (elevation <= lidar['elevation_max_deg']))
    return camera_mask, lidar_mask


def statistics(points, config, display_only=False):
    camera, lidar = coverage(points, config, display_only)
    return {'samples': len(points), 'both': int((camera & lidar).sum()),
            'camera_only': int((camera & ~lidar).sum()),
            'lidar_only': int((~camera & lidar).sum()),
            'neither': int((~camera & ~lidar).sum())}
