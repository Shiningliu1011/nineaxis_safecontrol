import numpy as np
from matplotlib import pyplot as plt

from .plot_style import configure_fonts


def pipe_pose_plot(data: dict, pose: dict) -> list:
    if "angular_jerk_W" not in pose:
        raise ValueError("PipePose:MissingAngularJerk 需要重新生成解析 angular_jerk_W")
    configure_fonts()
    figures = []
    figure = plt.figure(num="工具中心点轨迹与平滑工具轴", clear=True)
    axis = figure.add_subplot(projection="3d")
    points = np.asarray(data["position"])
    axis.plot(*points.T, color="black")
    indices = np.unique(np.rint(np.linspace(0, len(points) - 1, 45)).astype(int))
    axis.quiver(*points[indices].T, *pose["tool_axis_ref_W"][indices].T, length=5, color="blue")
    axis.set(xlabel="X 向位置 / mm", ylabel="Y 向位置 / mm", zlabel="Z 向位置 / mm", title="工具中心点轨迹与平滑工具轴")
    extent = np.ptp(points, axis=0)
    axis.set_box_aspect(np.maximum(extent, 1))
    axis.grid(True)
    figures.append(figure)
    figure, axis = plt.subplots(num="工具轴方向误差", clear=True)
    axis.plot(data["time"], np.rad2deg(pose["axis_smoothing_error"]))
    axis.set(xlabel="时间 / s", ylabel="方向误差 / °", title="平滑工具轴相对原始内法向的误差")
    axis.grid(True)
    figures.append(figure)
    for field, name, unit in (("omega_W", "角速度", "rad/s"), ("alpha_W", "角加速度", "rad/s²"), ("angular_jerk_W", "角加加速度", "rad/s³")):
        time, values = data["time"], pose[field]
        if field == "angular_jerk_W" and "angular_jerk_plot_time" in pose:
            time, values = pose["angular_jerk_plot_time"], pose["angular_jerk_plot_W"]
        figure, axis = plt.subplots(num=f"{name}模长", clear=True)
        axis.plot(time, np.linalg.norm(values, axis=1), color="red", linewidth=1.2)
        axis.set(xlabel="时间 / s", ylabel=f"{name}模长 / ({unit})", title=f"{name}模长")
        axis.grid(True)
        figures.append(figure)
        figure, axes = plt.subplots(3, 1, sharex=True, num=f"XYZ {name}分量", clear=True)
        for coordinate, (axis, color, label) in enumerate(zip(axes, ("red", "green", "blue"), ("X", "Y", "Z"))):
            axis.plot(time, values[:, coordinate], color=color, linewidth=1.2)
            axis.set_ylabel(f"{label} 向{name} / ({unit})")
            axis.grid(True)
        axes[0].set_title(f"XYZ {name}分量")
        axes[-1].set_xlabel("时间 / s")
        figures.append(figure)
    return figures
