from __future__ import annotations

import argparse
from importlib.metadata import version
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.ft2font import FT2Font
from matplotlib.text import Text
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from PIL import Image
import trimesh

from .oscbf_trajectory import bootstrap_portable
from .static_experiment_config import content_hash
from .static_experiment_evaluation import evaluation_samples
from .static_experiment_recording import read_stream, write_json
from .static_experiment_provenance import relocate_manifest, validate_run_report


def _fonts():
    records, coverage = {}, {}
    for name in ("Times New Roman", "SimSun"):
        path = Path(font_manager.findfont(font_manager.FontProperties(family=name), fallback_to_default=False))
        face = FT2Font(str(path))
        if face.family_name != name:
            raise ValueError(f"图表字体身份不一致：{face.family_name}")
        records[name] = {"path": str(path), "sha256": content_hash(path), "family": face.family_name}
        coverage[name] = set(face.get_charmap())
    matplotlib.rcParams.update({
        "font.family": ["Times New Roman", "SimSun"], "font.size": 11,
        "axes.unicode_minus": True, "figure.dpi": 120, "savefig.dpi": 200,
    })
    return records, coverage


def _save(figure, path: Path, coverage):
    for text in figure.findobj(match=Text):
        for character in text.get_text():
            name = "SimSun" if "\u3000" <= character <= "\uffff" else "Times New Roman"
            if not character.isspace() and ord(character) not in coverage[name]:
                raise ValueError(f"{name} 缺少图表字符：{character}")
    figure.savefig(path, bbox_inches="tight")
    plt.close(figure)
    with Image.open(path) as image:
        width, height = image.size
        image.verify()
    if width < 600 or height < 400:
        raise ValueError("图表导出尺寸不足")
    return {"path": str(path), "sha256": content_hash(path), "width": width, "height": height}


def _finish_axes(axes):
    for axis in np.asarray(axes).ravel():
        axis.grid(alpha=.2)
        axis.set_xlabel("仿真时间 (s)")


def _body_plot(axis, vertices, faces, color, alpha):
    axis.add_collection3d(Poly3DCollection(
        vertices[faces], facecolor=color, edgecolor="none", alpha=alpha, rasterized=True,
    ))


def plot_run(directory: Path, resource_root: Path, output: Path | None = None):
    directory, resource_root = directory.resolve(strict=True), resource_root.resolve(strict=True)
    output = (output or resource_root / "picture/static-obstacle-experiments" / directory.name).resolve()
    if not output.is_relative_to(resource_root / "picture"):
        raise ValueError("实验图表必须保存在仓库 picture 目录")
    manifest, report = validate_run_report(directory)
    with evaluation_samples(directory, resource_root, report) as (data_directory, data_report):
        return _plot_run(directory, resource_root, output, manifest, report, data_directory, data_report)


def _plot_run(directory, resource_root, output, manifest, report, data_directory, data_report):
    manifest = relocate_manifest(manifest, resource_root)
    data_hashes = data_report["derived_data_sha256"]
    for filename, expected in data_hashes.items():
        if content_hash(data_directory / filename) != expected:
            raise ValueError(f"图表测量数据身份不一致：{filename}")
    if content_hash(directory / "path.npz") != report["path_sha256"]:
        raise ValueError("图表路径身份不一致")
    for role in ("plant", "controller"):
        if content_hash(directory / f"{role}.sqlite3") != report["records_sha256"][role]:
            raise ValueError("原始数据与反馈报告身份不一致")
    with np.load(data_directory / "feedback_metrics.npz", allow_pickle=False) as data:
        metrics = {name: data[name] for name in data.files}
    with np.load(data_directory / "clearance_metrics.npz", allow_pickle=False) as data:
        clearances = {name: data[name] for name in data.files}
    with np.load(directory / "path.npz", allow_pickle=False) as data:
        path = {name: data[name] for name in data.files}
    feedback = read_stream(directory / "plant.sqlite3", "feedback")
    times = np.array([row["simulation_s"] for row in feedback])
    indices = np.searchsorted(times, metrics["simulation_s"])
    if not np.allclose(times[indices], metrics["simulation_s"], atol=1e-10, rtol=0):
        raise ValueError("关节反馈与图表测量时间不一致")
    q = np.array([feedback[index]["values"]["q"] for index in indices])
    cycles = read_stream(directory / "controller.sqlite3", "cycle")
    fonts, coverage = _fonts()
    output.mkdir(parents=True, exist_ok=False)
    files = []
    time_s = metrics["simulation_s"] - metrics["simulation_s"][0]

    figure, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True, layout="constrained")
    axes[0].plot(time_s, metrics["path_distance_m"] * 1000, label="路径距离")
    axes[0].plot(time_s, metrics["reference_position_error_m"] * 1000, label="参考位置误差", alpha=.7)
    axes[0].set_ylabel("位置误差 (mm)")
    axes[0].legend()
    axes[1].plot(time_s, np.rad2deg(metrics["tool_axis_angle_rad"]), color="#c55338")
    axes[1].set_ylabel("真实工具轴夹角 (°)")
    _finish_axes(axes)
    files.append(_save(figure, output / "tracking-errors.png", coverage))

    figure, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True, layout="constrained")
    feed = np.r_[0., np.diff(metrics["reference_progress_m"]) / np.diff(metrics["simulation_s"])]
    axes[0].plot(time_s, feed * 1000, label="获准实际进给")
    axes[0].plot(time_s, np.interp(
        metrics["reference_progress_m"], path["arc_length_m"], path["feedrate_m_s"],
    ) * 1000, label="名义进给", alpha=.6)
    axes[0].set_ylabel("路径进给 (mm/s)")
    axes[0].legend()
    phases = ("RUNNING", "REFERENCE_PAUSED", "RECOVERING", "ENDPOINT_HOLD", "FAULT")
    phase_values = np.array([phases.index(value) for value in metrics["phase"]])
    axes[1].step(time_s, phase_values, where="post")
    axes[1].set_yticks(range(5), ["跟随", "暂停", "恢复确认", "终点保持", "故障"])
    axes[1].set_ylabel("实际执行阶段")
    _finish_axes(axes)
    files.append(_save(figure, output / "feed-and-phase.png", coverage))

    figure, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True, layout="constrained")
    for axis, kind, label, required in zip(axes, ("external", "self"), ("外部间距", "自碰撞间距"), (.03, .01)):
        mask = clearances["kind"] == kind
        if not np.any(mask):
            raise ValueError(f"缺少 {kind} 实际间距")
        axis.plot(time_s, clearances["distance_m"][:, mask].min(axis=1) * 1000, label="全部对象中的最小距离")
        axis.axhline(required * 1000, color="#c55338", linestyle="--", label="要求间距")
        axis.set_ylabel(f"{label} (mm)")
        axis.legend()
    _finish_axes(axes)
    files.append(_save(figure, output / "clearances.png", coverage))

    limits = manifest["execution_limits"]
    lower, upper = np.asarray(limits["lower"]), np.asarray(limits["upper"])
    margin = np.minimum(q - lower, upper - q) / (upper - lower)
    figure, axis = plt.subplots(figsize=(9, 4.5), layout="constrained")
    for joint in range(9):
        axis.plot(time_s, margin[:, joint], label=f"J{joint + 1}", linewidth=.8)
    axis.axhline(0., color="#c55338", linestyle="--")
    axis.set_ylabel("归一化关节余量")
    axis.legend(ncol=5)
    _finish_axes([axis])
    files.append(_save(figure, output / "joint-margins.png", coverage))

    figure, axis = plt.subplots(figsize=(7, 4.5), layout="constrained")
    durations = np.sort([row["values"]["duration_s"] * 1000 for row in cycles])
    axis.step(durations, np.arange(1, len(durations) + 1) / len(durations) * 100, where="post")
    period_ms = manifest["config"]["control"]["period_s"] * 1000
    axis.axvline(.8 * period_ms, color="#c55338", linestyle="--", label="周期的 80%")
    axis.axvline(period_ms, color="black", linestyle=":", label="完整周期")
    axis.set_xlabel("计算与准入耗时 (ms)")
    axis.set_ylabel("累计比例 (%)")
    axis.grid(alpha=.2)
    axis.legend()
    files.append(_save(figure, output / "cycle-duration.png", coverage))

    bootstrap_portable(resource_root / "portable_oscbf")
    from work.nineaxis_kinematics import NineaxisKinematics
    from work.static_collision_meshes import load_meshes

    meshes = load_meshes(manifest["scene"]["self_collision"])
    kinematics = NineaxisKinematics()
    figure = plt.figure(figsize=(15, 5), layout="constrained")
    representative_samples = []
    for index, fraction in enumerate((.25, .5, .75)):
        selected = int(np.argmin(abs(metrics["projected_progress_m"] - fraction * path["arc_length_m"][-1])))
        actual_fraction = float(metrics["projected_progress_m"][selected] / path["arc_length_m"][-1])
        representative_samples.append({
            "sample_index": selected, "actual_fraction": actual_fraction, "target_fraction": fraction,
        })
        transforms = kinematics.forward_kinematics(q[selected])
        axis = figure.add_subplot(1, 3, index + 1, projection="3d")
        world = []
        for mesh in meshes:
            transform = transforms[mesh.link]
            vertices = mesh.vertices @ transform[:3, :3].T + transform[:3, 3]
            world.append(vertices)
            _body_plot(axis, vertices, mesh.faces, "#537894", .75)
        scene = manifest["scene"]
        for obstacle in scene["obstacles"]:
            sphere = trimesh.creation.icosphere(subdivisions=2, radius=obstacle["radius_m"])
            vertices = sphere.vertices + obstacle["center_m"]
            _body_plot(axis, vertices, sphere.faces, "#d55d41", .5)
            box = trimesh.creation.box(extents=[2 * obstacle["radius_m"]] * 3)
            _body_plot(axis, box.vertices + obstacle["center_m"], box.faces, "#d55d41", .1)
            world.extend([vertices, box.vertices + obstacle["center_m"]])
        cylinder = scene["workpiece"]
        angles, heights = np.meshgrid(np.linspace(0., 2 * np.pi, 40), [-cylinder["height_m"] / 2, cylinder["height_m"] / 2])
        cylinder_points = np.stack([
            cylinder["center_m"][0] + cylinder["radius_m"] * np.cos(angles),
            cylinder["center_m"][1] + heights,
            cylinder["center_m"][2] + cylinder["radius_m"] * np.sin(angles),
        ], axis=-1)
        axis.plot_surface(*np.moveaxis(cylinder_points, -1, 0), alpha=.15, color="#999999")
        table = trimesh.creation.box(extents=2 * np.array(scene["table"]["half_extents_m"]))
        _body_plot(axis, table.vertices + scene["table"]["center_m"], table.faces, "#888888", .2)
        world.extend([cylinder_points.reshape(-1, 3), table.vertices + scene["table"]["center_m"]])
        axis.plot(*path["positions_m"].T, color="#e0a127", linewidth=1., label="参考路径")
        axis.plot(*metrics["positions_m"].T, color="#267d63", linewidth=.8, label="实际轨迹")
        points = np.vstack([*world, path["positions_m"]])
        center, extent = (points.max(axis=0) + points.min(axis=0)) / 2, np.ptp(points, axis=0).max() * .55
        axis.set_xlim(center[0] - extent, center[0] + extent)
        axis.set_ylim(center[1] - extent, center[1] + extent)
        axis.set_zlim(center[2] - extent, center[2] + extent)
        axis.set_box_aspect((1, 1, 1))
        axis.view_init(elev=20, azim=-55, vertical_axis="y")
        axis.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)", title=f"{actual_fraction:.1%} 路径进度")
        axis.legend(loc="upper left")
    files.append(_save(figure, output / "representative-configurations.png", coverage))
    record = {
        "run": str(directory), "report_sha256": content_hash(directory / "feedback_report.json"),
        "renderer_sha256": content_hash(Path(__file__)),
        "inputs_sha256": data_hashes,
        "evaluation_provenance": {
            name: data_report.get(name)
            for name in ("evaluator_sha256", "evaluation_sources_sha256")
        },
        "representative_samples": representative_samples,
        "fonts": fonts, "files": files, "versions": {
            name: version(name) for name in ("numpy", "scipy", "matplotlib", "Pillow", "trimesh")
        },
    }
    write_json(output / "figures.json", record)
    return record


def main(argv=None):
    parser = argparse.ArgumentParser(description="根据静态实验实际反馈导出图表")
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--resource-root", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    result = plot_run(args.run, args.resource_root, args.output)
    print(f"已导出 {len(result['files'])} 张图表", flush=True)


if __name__ == "__main__":
    main()
