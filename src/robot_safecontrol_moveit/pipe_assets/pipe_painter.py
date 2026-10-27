import tkinter as tk
from copy import deepcopy
from pathlib import Path
from tkinter import filedialog

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, CheckButtons, RadioButtons, TextBox

from .path_io import export_path, load_editable_path
from .pipe_demo_model import pipe_demo_model
from .pipe_examples import example_path
from .pipe_path_curvature import pipe_path_curvature
from .pipe_prepare_model import pipe_prepare_model
from .pipe_reference_path import pipe_reference_path
from .pipe_surface_eval import pipe_surface_eval
from .plot_style import configure_fonts


def _defaults(options):
    values = {
        "OffsetMM": 5.0,
        "SampleSpacingMM": 1.0,
        "ThetaRangeDeg": np.array([-180.0, 540.0]),
        "Visible": "on",
        "PathDirectory": Path.cwd() / ".scratch/pipe-assets/paths",
        "ShowCurvature": False,
    }
    unknown = set(options) - set(values)
    if unknown:
        raise ValueError(f"未知画笔选项：{sorted(unknown)}")
    values.update(options)
    for name, positive in (("OffsetMM", False), ("SampleSpacingMM", True)):
        value = np.asarray(values[name])
        if value.ndim or not np.isrealobj(value) or not np.isfinite(value):
            raise ValueError(f"{name} 必须为有限实数")
        if value < 0 or (positive and value == 0):
            raise ValueError(f"{name} 超出有效范围")
        values[name] = float(value)
    limits = np.asarray(values["ThetaRangeDeg"], dtype=float).reshape(-1)
    if limits.size != 2 or not np.all(np.isfinite(limits)) or not 0 < np.diff(limits)[0] <= 3600:
        raise ValueError("ThetaRangeDeg 必须递增且跨度不超过 3600 deg")
    values["ThetaRangeDeg"] = limits
    if values["Visible"] not in ("on", "off"):
        raise ValueError("Visible 必须为 on 或 off")
    if values["ShowCurvature"] not in (True, False):
        raise ValueError("ShowCurvature 必须为布尔值")
    values["PathDirectory"] = Path(values["PathDirectory"]).resolve()
    return values


class PipePainter:
    def __init__(self, model, options):
        self.options = _defaults(options)
        self.model = pipe_prepare_model(model)
        self.pathDirectory = self.options["PathDirectory"]
        self.pathDirectory.mkdir(parents=True, exist_ok=True)
        self.offsetMM = self.options["OffsetMM"]
        self.sampleSpacingMM = self.options["SampleSpacingMM"]
        self.thetaRangeDeg = self.options["ThetaRangeDeg"].copy()
        self.anchors = np.empty((0, 2))
        self.pathDefinition = {"type": "c2-spline"}
        self.history = []
        self.drawing = False
        self.dragging = False
        self.viewAdjusting = False
        self._dragIndex = None
        self._rotationStart = None
        self._panStart = None
        self._strokeBackup = None
        self._strokeStartIndex = 0
        self._drawMode = 0
        self._canvasFigure = None
        self.CurvatureFigure = None
        self._layers = [True, True, True]
        self._widgets = []
        self._detachedWidgets = []
        self._modeControls = []
        self._navigationControls = []
        self._mesh = []
        self._realSurfaces = []
        self._virtualSurfaces = []
        configure_fonts()
        self.Figure = plt.figure(figsize=(14.8, 8.5))
        self.Figure.canvas.manager.set_window_title("管面轨迹画笔")
        self.Figure.set_visible(self.options["Visible"] == "on")
        self.ax2 = self.Figure.add_axes((0.25, 0.23, 0.34, 0.62))
        self._mainCanvas = self.ax2
        self.ax3 = self.Figure.add_axes((0.65, 0.23, 0.32, 0.62), projection="3d")
        self.ax3.disable_mouse_rotation()
        self.ax3.view_init(elev=22, azim=-48)
        self.ax3.set(xlabel="X / mm", ylabel="Y / mm", zlabel="Z / mm")
        self.Figure.suptitle(f"管面轨迹画笔 / {self.model['name']}")
        self._status = self.Figure.text(0.025, 0.025, "")
        self._initialize_canvas()
        self._path3, = self.ax3.plot([], [], [], color="firebrick", linewidth=2)
        self._start3, = self.ax3.plot([], [], [], "o", color="seagreen")
        self._end3, = self.ax3.plot([], [], [], "s", color="firebrick")
        for left, right in zip(self.model["breaks"][:-1], self.model["breaks"][1:]):
            x, angle = np.meshgrid(np.linspace(left, right, 35), np.linspace(0, 360, 49), indexing="ij")
            uv = np.column_stack((x.ravel(), angle.ravel()))
            _, points, normals, _, _ = pipe_surface_eval(self.model, uv, 0)
            self._mesh.append((points, normals, x.shape))
        self._draw_surfaces()
        self._create_controls()
        self._connect(self.Figure)
        self._refresh()
        if self.options["ShowCurvature"]:
            self.showCurvature()

    def _initialize_canvas(self):
        self.ax2.set(xlim=(0, self.model["totalLength"]), ylim=self.thetaRangeDeg,
                     xlabel="沿管位置 / mm", ylabel="连续周向角 / deg")
        self.ax2.grid(alpha=0.3)
        colors = ("#eaf2fc", "#f5efe1", "#e7f5ef")
        self._regionLabels = []
        for index, segment in enumerate(self.model["segments"]):
            left, right = self.model["breaks"][index:index + 2]
            self.ax2.axvspan(left, right, color=colors[index % 3], zorder=0)
            self._regionLabels.append(self.ax2.text(
                (left + right) / 2, self.thetaRangeDeg[1], segment["name"],
                ha="center", va="top", clip_on=True,
            ))
        self._path2, = self.ax2.plot([], [], color="firebrick", linewidth=1.8)
        self._keys2, = self.ax2.plot([], [], "o", color="firebrick", mfc="white", ms=4)
        self._start2, = self.ax2.plot([], [], "o", color="seagreen")
        self._end2, = self.ax2.plot([], [], "s", color="firebrick")

    def _draw_surfaces(self):
        for item in self._realSurfaces + self._virtualSurfaces:
            item.remove()
        self._realSurfaces, self._virtualSurfaces = [], []
        for points, normals, shape in self._mesh:
            virtual = points + self.offsetMM * normals
            self._realSurfaces.append(self.ax3.plot_surface(
                *[points[:, index].reshape(shape) for index in range(3)],
                color="#969ba3", linewidth=0, alpha=1,
            ))
            self._virtualSurfaces.append(self.ax3.plot_surface(
                *[virtual[:, index].reshape(shape) for index in range(3)],
                color="#197bd1", linewidth=0, alpha=0.25,
            ))
        self._fit_limits()

    def _fit_limits(self):
        all_points = np.vstack([
            values for points, normals, _ in self._mesh
            for values in (points, points + self.offsetMM * normals)
        ])
        low, high = np.min(all_points, axis=0), np.max(all_points, axis=0)
        padding = np.maximum((high - low) * 0.12, 5)
        self.ax3.set(xlim=(low[0] - padding[0], high[0] + padding[0]),
                     ylim=(low[1] - padding[1], high[1] + padding[1]),
                     zlim=(low[2] - padding[2], high[2] + padding[2]))
        self.ax3.set_box_aspect(np.maximum(high - low, 1))

    def _control(self, cls, rectangle, *args, **kwargs):
        host = kwargs.pop("figure", self.Figure)
        widget = cls(host.add_axes(rectangle), *args, **kwargs)
        if host is self.Figure:
            self._widgets.append(widget)
        else:
            self._detachedWidgets.append(widget)
        return widget

    def _create_controls(self):
        modes = self._control(RadioButtons, (0.025, 0.73, 0.18, 0.17),
                              ("重新画线", "从终点续画", "拖动关键点", "点击添加"))
        modes.on_clicked(lambda label: self.setDrawMode(modes.labels.index(
            next(item for item in modes.labels if item.get_text() == label))))
        self._modeControls.append(modes)
        self._offsetControl = self._control(TextBox, (0.08, 0.665, 0.12, 0.04), "距离 mm ", initial=str(self.offsetMM))
        self._offsetControl.on_submit(lambda value: self.setOffset(float(value)))
        self._spacingControl = self._control(TextBox, (0.08, 0.60, 0.12, 0.04), "间隔 mm ", initial=str(self.sampleSpacingMM))
        self._spacingControl.on_submit(lambda value: self.setSpacing(float(value)))
        self._thetaControl = self._control(TextBox, (0.08, 0.535, 0.12, 0.04), "范围 deg ",
                                          initial=f"{self.thetaRangeDeg[0]:g},{self.thetaRangeDeg[1]:g}")
        self._thetaControl.on_submit(lambda value: self.setThetaRange(np.fromstring(value, sep=",")))
        layers = self._control(CheckButtons, (0.025, 0.405, 0.18, 0.10), ("真实曲面", "距离曲面", "路径"), self._layers)
        layers.on_clicked(lambda label: self.setLayer(
            {"真实曲面": "real", "距离曲面": "virtual", "路径": "path"}[label],
            layers.get_status()[[item.get_text() for item in layers.labels].index(label)],
        ))
        actions = (
            ("撤销", self.undo), ("清空", lambda: self.setPath([])),
            ("示例路径", self.setExamplePath), ("简化笔迹", self.simplifyPath),
            ("导出", self._export_dialog), ("载入", self._load_dialog),
            ("查看曲率", self.showCurvature), ("恢复视角", self.resetView),
        )
        for index, (label, action) in enumerate(actions):
            button = self._control(Button, (0.025 + 0.095 * (index % 2), 0.34 - 0.065 * (index // 2), 0.085, 0.045), label)
            button.on_clicked(lambda event, callback=action: callback())
        navigation = self._control(Button, (0.25, 0.89, 0.16, 0.045), "调整视图")
        navigation.on_clicked(lambda event: self.setViewAdjustment(not self.viewAdjusting))
        self._navigationControls.append(navigation)
        full_view = self._control(Button, (0.43, 0.89, 0.10, 0.045), "二维全览")
        full_view.on_clicked(lambda event: self.resetCanvasView())
        dock = self._control(Button, (0.55, 0.89, 0.10, 0.045), "收回画布")
        dock.on_clicked(lambda event: self.dockCanvas())
        focus = self._control(TextBox, (0.76, 0.89, 0.10, 0.045), "管段序号 ", initial="1")
        focus.on_submit(lambda value: self.focusSegment(int(value)))

    def _sync_controls(self):
        for widget, text in (
            (self._offsetControl, f"{self.offsetMM:g}"),
            (self._spacingControl, f"{self.sampleSpacingMM:g}"),
            (self._thetaControl, f"{self.thetaRangeDeg[0]:g},{self.thetaRangeDeg[1]:g}"),
        ):
            if widget.text != text:
                enabled = widget.eventson
                widget.eventson = False
                try:
                    widget.set_val(text)
                finally:
                    widget.eventson = enabled

    def _update_canvas_decorations(self):
        xr, yr = self.ax2.get_xlim(), self.ax2.get_ylim()
        for index, label in enumerate(self._regionLabels):
            left = max(xr[0], self.model["breaks"][index])
            right = min(xr[1], self.model["breaks"][index + 1])
            label.set_visible(right > left)
            label.set_position(((left + right) / 2, yr[1] - 0.035 * np.ptp(yr)))
            label.set_text(self.model["segments"][index]["name"] if len(self.model["segments"]) <= 6
                           or np.ptp(xr) < self.model["totalLength"] * 0.25 else f"{index + 1:02d}")

    def _connect(self, figure):
        for name, method in (
            ("button_press_event", self._mouse_down), ("motion_notify_event", self._mouse_move),
            ("button_release_event", self._mouse_up), ("scroll_event", self._mouse_wheel),
            ("key_press_event", self._key_press),
        ):
            figure.canvas.mpl_connect(name, method)
        figure.canvas.mpl_connect("close_event", lambda event: self._window_closed(event.canvas.figure))

    def _window_closed(self, figure):
        if figure is self._canvasFigure:
            self.dockCanvas()
        elif figure is self.Figure:
            self.close()

    def _validate_points(self, points):
        points = np.asarray(points)
        if not points.size:
            return np.empty((0, 2))
        if not np.isrealobj(points):
            raise ValueError("关键点必须为实数")
        if points.ndim == 1 and points.shape != (2,):
            raise ValueError("关键点必须包含两列")
        if points.ndim not in (1, 2) or (points.ndim == 2 and points.shape[1] != 2):
            raise ValueError("关键点必须为 N×2 数组")
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        if not np.all(np.isfinite(points)) or np.any(points[:, 0] < 0) or np.any(points[:, 0] > self.model["totalLength"]):
            raise ValueError("关键点超出管道画布范围")
        if np.any(points[:, 1] < self.thetaRangeDeg[0]) or np.any(points[:, 1] > self.thetaRangeDeg[1]):
            raise ValueError("关键点超出角度范围")
        return points.copy()

    def _push_history(self):
        self.history.append((self.anchors.copy(), self.thetaRangeDeg.copy(), deepcopy(self.pathDefinition)))
        self.history = self.history[-30:]

    def _refresh(self):
        self.preview = pipe_reference_path(self.model, self.anchors, self.offsetMM, None, self.pathDefinition)
        uv, points = self.preview["canvasUV"], self.preview["tcpPoints"]
        self._path2.set_data(uv[:, 0], uv[:, 1])
        self._keys2.set_data(self.anchors[:, 0], self.anchors[:, 1])
        self._path3.set_data_3d(*points.T)
        if points.size:
            self._start2.set_data(uv[[0], 0], uv[[0], 1])
            self._end2.set_data(uv[[-1], 0], uv[[-1], 1])
            self._start3.set_data_3d(*points[[0]].T)
            self._end3.set_data_3d(*points[[-1]].T)
        else:
            self._start2.set_data([], [])
            self._end2.set_data([], [])
            self._start3.set_data_3d([], [], [])
            self._end3.set_data_3d([], [], [])
        length = self.preview["arcLengthMM"][-1] if points.size else 0
        self._status.set_text(f"关键点 {len(self.anchors)} | 路径 {length:.3f} mm | 工作距离 {self.offsetMM:g} mm")
        self._apply_visibility()
        self._update_canvas_decorations()
        if self.getState()["curvatureVisible"]:
            if self.drawing or self.dragging:
                self._curvatureLine.set_data([], [])
                self._curvatureAxes.set_title("正在编辑路径，结束编辑后更新曲率")
            else:
                self._refresh_curvature()
        self._draw_idle()

    def _draw_idle(self):
        if self.options["Visible"] == "on":
            self.Figure.canvas.draw_idle()
            if self._canvasFigure is not None:
                self._canvasFigure.canvas.draw_idle()

    def _apply_visibility(self):
        for layer, artists in zip(self._layers, (
            self._realSurfaces, self._virtualSurfaces,
            [self._path2, self._keys2, self._start2, self._end2, self._path3, self._start3, self._end3],
        )):
            for artist in artists:
                artist.set_visible(layer)

    def setPath(self, points, definition=None):
        candidate = self._validate_points(points)
        definition = {"type": "c2-spline"} if definition is None else definition
        pipe_reference_path(self.model, candidate, self.offsetMM, None, definition)
        self._push_history()
        self.anchors, self.pathDefinition = candidate, deepcopy(definition)
        self._refresh()

    def setOffset(self, value):
        candidate = _defaults({"OffsetMM": value})["OffsetMM"]
        pipe_reference_path(self.model, self.anchors, candidate, None, self.pathDefinition)
        self.offsetMM = candidate
        self._sync_controls()
        self._draw_surfaces()
        self._refresh()

    def setSpacing(self, value):
        self.sampleSpacingMM = _defaults({"SampleSpacingMM": value})["SampleSpacingMM"]
        self._sync_controls()

    def setThetaRange(self, value):
        limits = _defaults({"ThetaRangeDeg": value})["ThetaRangeDeg"]
        if self.anchors.size and np.any((self.anchors[:, 1] < limits[0]) | (self.anchors[:, 1] > limits[1])):
            raise ValueError("新角度范围没有包含已有关键点")
        self.thetaRangeDeg = limits
        self._sync_controls()
        self.ax2.set_ylim(limits)
        self._refresh()

    def setDrawMode(self, index):
        if index not in range(4):
            raise ValueError("绘图模式必须位于 0–3")
        self.endStroke()
        self.dragging = False
        self._drawMode = index
        for widget in self._modeControls:
            if widget.value_selected != widget.labels[index].get_text():
                enabled = widget.eventson
                widget.eventson = False
                try:
                    widget.set_active(index)
                finally:
                    widget.eventson = enabled

    def beginStroke(self, point, append=False):
        point = self._validate_points(point)
        if point.shape != (1, 2) or self.drawing:
            raise ValueError("笔画需要一个起点，且当前没有进行中的笔画")
        if self.viewAdjusting:
            raise ValueError("调整视图结束后才能绘画")
        self._strokeBackup = (self.anchors.copy(), deepcopy(self.pathDefinition))
        self._push_history()
        self.pathDefinition = {"type": "c2-spline"}
        self._strokeStartIndex = max(0, len(self.anchors) - 1) if append else 0
        self.anchors = np.vstack((self.anchors, point)) if append else point
        self.drawing = True
        self._refresh()

    def _pixels(self, points):
        return np.asarray(points) / [np.ptp(self.ax2.get_xlim()), np.ptp(self.ax2.get_ylim())] * self.ax2.bbox.size

    def appendStroke(self, point):
        if not self.drawing:
            return
        point = self._validate_points(point)
        if point.shape != (1, 2):
            raise ValueError("每次追加一个关键点")
        if np.linalg.norm(self._pixels(point[0] - self.anchors[-1])) > 0.05:
            self.anchors = np.vstack((self.anchors, point))
            self._refresh()

    def _simplified(self, points, tolerance):
        if len(points) < 3:
            return points.copy()
        pixels = self._pixels(points)
        keep = np.zeros(len(points), dtype=bool)
        keep[[0, -1]] = True
        intervals = [(0, len(points) - 1)]
        while intervals:
            left, right = intervals.pop()
            if right <= left + 1:
                continue
            chord = pixels[right] - pixels[left]
            delta = pixels[left + 1:right] - pixels[left]
            if chord @ chord < np.finfo(float).eps:
                distances = np.linalg.norm(delta, axis=1)
            else:
                parameter = np.clip(delta @ chord / (chord @ chord), 0, 1)
                distances = np.linalg.norm(delta - parameter[:, None] * chord, axis=1)
            middle = int(np.argmax(distances))
            if distances[middle] > tolerance:
                middle += left + 1
                keep[middle] = True
                intervals.extend(((left, middle), (middle, right)))
        return points[keep].copy()

    def endStroke(self):
        if self.drawing:
            self.drawing = False
            index = self._strokeStartIndex
            self.anchors = np.vstack((self.anchors[:index], self._simplified(self.anchors[index:], 2)))
            self._refresh()

    def cancelStroke(self):
        if self.drawing or self.dragging:
            self.anchors, self.pathDefinition = self._strokeBackup
            self.drawing = self.dragging = False
            self.history.pop()
            self._refresh()

    def moveKeypoint(self, index, point):
        candidate = self._validate_points(point)
        if candidate.shape != (1, 2) or not isinstance(index, (int, np.integer)) or not 1 <= index <= len(self.anchors):
            raise ValueError("关键点序号或位置无效")
        points = self.anchors.copy()
        points[index - 1] = candidate[0]
        self.setPath(points)

    def undo(self):
        if self.history:
            self.anchors, self.thetaRangeDeg, self.pathDefinition = self.history.pop()
            self.drawing = self.dragging = False
            self.ax2.set_ylim(self.thetaRangeDeg)
            self._sync_controls()
            self._refresh()

    def simplifyPath(self):
        if len(self.anchors) >= 3:
            self.setPath(self._simplified(self.anchors, 6))

    def setExamplePath(self):
        points, definition = example_path(self.model, self.thetaRangeDeg)
        self.setPath(points, definition)

    def setLayer(self, layer, value):
        if layer not in ("real", "virtual", "path") or value not in (True, False):
            raise ValueError("图层必须为 real/virtual/path，状态必须为布尔值")
        self._layers[("real", "virtual", "path").index(layer)] = bool(value)
        self._apply_visibility()
        self._draw_idle()

    def getState(self):
        return {
            "model": self.model, "anchors": self.anchors.copy(), "preview": deepcopy(self.preview) if hasattr(self, "preview") else None,
            "offsetMM": self.offsetMM, "sampleSpacingMM": self.sampleSpacingMM,
            "thetaRangeDeg": self.thetaRangeDeg.copy(), "pathDirectory": str(self.pathDirectory),
            "pathDefinition": deepcopy(self.pathDefinition),
            "curvatureVisible": self.CurvatureFigure is not None and plt.fignum_exists(self.CurvatureFigure.number),
            "layers": self._layers.copy(), "drawing": self.drawing,
            "canvasDetached": self._canvasFigure is not None and plt.fignum_exists(self._canvasFigure.number),
            "viewAdjusting": self.viewAdjusting,
            "canvasXLim": np.array(self.ax2.get_xlim()), "canvasYLim": np.array(self.ax2.get_ylim()),
        }

    def getSamples(self):
        if len(self.anchors) < 2:
            raise ValueError("路径至少需要两个不同关键点")
        result = pipe_reference_path(self.model, self.anchors, self.offsetMM, self.sampleSpacingMM, self.pathDefinition)
        if len(result["tcpPoints"]) < 2 or result["arcLengthMM"][-1] < 1e-8:
            raise ValueError("路径长度为零")
        return result

    def getCurvature(self):
        return pipe_path_curvature(self.model, self.anchors, self.offsetMM, self.pathDefinition)

    def showCurvature(self):
        if not self.getState()["curvatureVisible"]:
            self.CurvatureFigure, self._curvatureAxes = plt.subplots(figsize=(8.8, 4.7))
            self.CurvatureFigure.set_visible(self.options["Visible"] == "on")
            self.CurvatureFigure.canvas.manager.set_window_title("TCP 路径曲率")
            self._curvatureLine, = self._curvatureAxes.plot([], [], color="#175caa")
            self._curvatureAxes.set(xlabel="累计路径长度 / mm", ylabel="曲率 / mm⁻¹")
            self._curvatureAxes.grid(alpha=0.3)
        self._refresh_curvature()

    def _refresh_curvature(self):
        data = self.getCurvature()
        self._curvatureLine.set_data(data["arcLengthMM"], data["kappa"])
        self._curvatureAxes.relim()
        self._curvatureAxes.autoscale_view()
        self._curvatureAxes.set_title(f"TCP 路径曲率 / 工作距离 {self.offsetMM:g} mm")
        if self.options["Visible"] == "on":
            self.CurvatureFigure.canvas.draw_idle()

    def _resolve_file(self, filename):
        filename = Path(filename)
        return self.pathDirectory / filename if filename.parent == Path(".") else filename

    def export(self, stem):
        stem = self._resolve_file(stem)
        trajectory = self.getSamples()
        settings = {"OffsetMM": self.offsetMM, "SampleSpacingMM": self.sampleSpacingMM, "ThetaRangeDeg": self.thetaRangeDeg}
        return export_path(self.model, trajectory, settings, stem)

    def load(self, filename):
        saved = load_editable_path(self.model, self._resolve_file(filename))
        options = _defaults(saved["settings"])
        points = saved["trajectory"]["keypointsCanvas"]
        limits = options["ThetaRangeDeg"]
        if np.any((points[:, 1] < limits[0]) | (points[:, 1] > limits[1])):
            raise ValueError("路径文件的关键点超出角度范围")
        definition = saved["trajectory"].get("pathDefinition", {"type": "c2-spline"})
        pipe_reference_path(self.model, points, options["OffsetMM"], options["SampleSpacingMM"], definition)
        self._push_history()
        self.anchors, self.pathDefinition = points, definition
        self.thetaRangeDeg, self.sampleSpacingMM = limits, options["SampleSpacingMM"]
        self.ax2.set_ylim(limits)
        self.setOffset(options["OffsetMM"])

    def _export_dialog(self):
        self.getSamples()
        root = tk.Tk()
        root.withdraw()
        try:
            filename = filedialog.asksaveasfilename(initialdir=self.pathDirectory, initialfile="pipe_path.mat", defaultextension=".mat")
        finally:
            root.destroy()
        if filename:
            self.export(filename)

    def _load_dialog(self):
        root = tk.Tk()
        root.withdraw()
        try:
            filename = filedialog.askopenfilename(initialdir=self.pathDirectory, filetypes=[("MAT", "*.mat")])
        finally:
            root.destroy()
        if filename:
            self.load(filename)

    def setViewAdjustment(self, enabled):
        if enabled not in (True, False):
            raise ValueError("视图状态必须为布尔值")
        self.endStroke()
        self.dragging = False
        self._panStart = None
        self.viewAdjusting = bool(enabled)
        if enabled and self._canvasFigure is None:
            old_limits = self.ax2.get_xlim(), self.ax2.get_ylim()
            self.ax2.set_visible(False)
            self._canvasFigure = plt.figure(figsize=(9, 7))
            self._canvasFigure.set_visible(self.options["Visible"] == "on")
            self._canvasFigure.canvas.manager.set_window_title("独立二维画布")
            self.ax2 = self._canvasFigure.add_axes((0.12, 0.13, 0.82, 0.65))
            self._initialize_canvas()
            self.ax2.set(xlim=old_limits[0], ylim=old_limits[1])
            self._create_detached_controls()
            self._connect(self._canvasFigure)
            self._refresh()
        for widget in self._navigationControls:
            widget.label.set_text("调整视图：开启" if self.viewAdjusting else "调整视图：关闭")
        self._draw_idle()

    def _create_detached_controls(self):
        figure = self._canvasFigure
        navigation = self._control(Button, (0.04, 0.94, 0.22, 0.04), "调整视图", figure=figure)
        navigation.on_clicked(lambda event: self.setViewAdjustment(not self.viewAdjusting))
        self._navigationControls.append(navigation)
        for position, label, action in (
            ((0.29, 0.94, 0.16, 0.04), "二维全览", self.resetCanvasView),
            ((0.48, 0.94, 0.16, 0.04), "收回画布", self.dockCanvas),
        ):
            button = self._control(Button, position, label, figure=figure)
            button.on_clicked(lambda event, callback=action: callback())
        focus = self._control(TextBox, (0.79, 0.94, 0.14, 0.04), "管段序号 ", initial="1", figure=figure)
        focus.on_submit(lambda value: self.focusSegment(int(value)))
        modes = self._control(RadioButtons, (0.04, 0.79, 0.90, 0.13),
                              ("重新画线", "从终点续画", "拖动关键点", "点击添加"),
                              active=self._drawMode, figure=figure)
        modes.on_clicked(lambda label: self.setDrawMode(
            [item.get_text() for item in modes.labels].index(label)))
        self._modeControls.append(modes)

    def dockCanvas(self):
        if self._canvasFigure is not None:
            self.endStroke()
            self.dragging = False
            self._panStart = None
            child = self._canvasFigure
            limits = self.ax2.get_xlim(), self.ax2.get_ylim()
            self._canvasFigure = None
            self.ax2 = self._mainCanvas
            self.ax2.clear()
            self.ax2.set_visible(True)
            self._initialize_canvas()
            self.ax2.set(xlim=limits[0], ylim=limits[1])
            plt.close(child)
            self.viewAdjusting = False
            self._detachedWidgets.clear()
            self._modeControls = self._modeControls[:1]
            self._navigationControls = self._navigationControls[:1]
            self._navigationControls[0].label.set_text("调整视图：关闭")
            self._refresh()

    def getCanvasFigure(self):
        return self.ax2.figure

    def _set_canvas_limits(self, xr, yr):
        limits = []
        for value, bounds in ((xr, [0, self.model["totalLength"]]), (yr, self.thetaRangeDeg)):
            full_width = np.ptp(bounds)
            width = np.clip(np.ptp(value), full_width * 1e-8, full_width)
            low = np.clip(np.mean(value) - width / 2, bounds[0], bounds[1] - width)
            limits.append([low, low + width])
        self.ax2.set(xlim=limits[0], ylim=limits[1])
        self._update_canvas_decorations()
        self._draw_idle()

    def focusSegment(self, index):
        if not isinstance(index, (int, np.integer)) or not 1 <= index <= len(self.model["segments"]):
            raise ValueError("管段序号无效")
        self.setViewAdjustment(True)
        bounds = self.model["breaks"][index - 1:index + 1]
        self._set_canvas_limits(bounds + [-0.08 * np.ptp(bounds), 0.08 * np.ptp(bounds)], self.ax2.get_ylim())

    def zoomCanvas(self, factor, point=None):
        if np.ndim(factor) or not np.isfinite(factor) or factor <= 0:
            raise ValueError("缩放倍数必须为有限正数")
        if self.viewAdjusting:
            xr, yr = np.asarray(self.ax2.get_xlim()), np.asarray(self.ax2.get_ylim())
            point = np.asarray([xr.mean(), yr.mean()] if point is None else point, dtype=float)
            if point.shape != (2,) or not np.all(np.isfinite(point)):
                raise ValueError("缩放中心需要两个有限坐标")
            self._set_canvas_limits(point[0] + (xr - point[0]) / factor, point[1] + (yr - point[1]) / factor)

    def panCanvas(self, fraction):
        fraction = np.asarray(fraction, dtype=float)
        if fraction.shape != (2,) or not np.all(np.isfinite(fraction)):
            raise ValueError("平移比例需要两个有限数值")
        if self.viewAdjusting:
            xr, yr = np.asarray(self.ax2.get_xlim()), np.asarray(self.ax2.get_ylim())
            self._set_canvas_limits(xr + fraction[0] * np.ptp(xr), yr + fraction[1] * np.ptp(yr))

    def resetCanvasView(self):
        self._set_canvas_limits([0, self.model["totalLength"]], self.thetaRangeDeg)

    def resetView(self):
        self.ax3.view_init(elev=22, azim=-48)
        self._fit_limits()
        self._draw_idle()

    def _mouse_down(self, event):
        if event.inaxes is self.ax3:
            self._rotationStart = (event.x, event.y, self.ax3.azim, self.ax3.elev)
            return
        if event.inaxes is not self.ax2 or event.xdata is None or event.ydata is None:
            return
        point = np.array([event.xdata, event.ydata])
        if self.viewAdjusting:
            if event.button == 1:
                self._panStart = (np.array([event.x, event.y]), np.asarray(self.ax2.get_xlim()), np.asarray(self.ax2.get_ylim()))
            return
        distance = np.linalg.norm(self._pixels(self.anchors - point), axis=1) if len(self.anchors) else np.array([])
        nearest = int(np.argmin(distance)) if distance.size else None
        if event.button == 3:
            if nearest is not None and distance[nearest] < 14:
                self.setPath(np.delete(self.anchors, nearest, axis=0))
            return
        if event.button != 1:
            return
        if self._drawMode in (0, 1):
            if self._drawMode == 1 and len(self.anchors) and np.linalg.norm(self._pixels(point - self.anchors[-1])) > 22:
                raise ValueError("续画起点需要靠近当前终点")
            self.beginStroke(point, self._drawMode == 1)
        elif self._drawMode == 2:
            if nearest is not None and distance[nearest] < 14:
                self._strokeBackup = (self.anchors.copy(), deepcopy(self.pathDefinition))
                self._push_history()
                self._dragIndex, self.dragging = nearest, True
        else:
            self.setPath(np.vstack((self.anchors, point)))

    def _mouse_move(self, event):
        if self._rotationStart is not None:
            x, y, azimuth, elevation = self._rotationStart
            self.ax3.view_init(elev=np.clip(elevation - 0.35 * (event.y - y), -85, 85), azim=azimuth - 0.35 * (event.x - x))
            self._draw_idle()
            return
        if self._panStart is not None and event.canvas is self.ax2.figure.canvas:
            previous, xr, yr = self._panStart
            delta = -(np.array([event.x, event.y]) - previous) / self.ax2.bbox.size * [np.ptp(xr), np.ptp(yr)]
            self._set_canvas_limits(xr + delta[0], yr + delta[1])
            return
        if self.dragging and event.canvas is self.ax2.figure.canvas:
            point = self.ax2.transData.inverted().transform([event.x, event.y])
            point = np.clip(point, [self.ax2.get_xlim()[0], self.ax2.get_ylim()[0]],
                            [self.ax2.get_xlim()[1], self.ax2.get_ylim()[1]])
            self.anchors[self._dragIndex] = self._validate_points(point)[0]
            self.pathDefinition = {"type": "c2-spline"}
            self._refresh()
            return
        if event.xdata is None or event.ydata is None or event.inaxes is not self.ax2:
            self.endStroke()
            return
        point = np.array([event.xdata, event.ydata])
        if self.drawing and np.linalg.norm(self._pixels(point - self.anchors[-1])) >= 2:
            self.appendStroke(point)

    def _mouse_up(self, event):
        self._rotationStart = self._panStart = None
        if self.drawing and event.inaxes is self.ax2 and event.xdata is not None:
            self.appendStroke([event.xdata, event.ydata])
        self.endStroke()
        if self.dragging:
            self.dragging = False
            self._refresh()

    def _mouse_wheel(self, event):
        if event.inaxes is self.ax2:
            self.zoomCanvas(1.2 ** event.step, [event.xdata, event.ydata])
        elif event.inaxes is self.ax3:
            factor = 1.12 ** (-event.step)
            for getter, setter in ((self.ax3.get_xlim, self.ax3.set_xlim), (self.ax3.get_ylim, self.ax3.set_ylim), (self.ax3.get_zlim, self.ax3.set_zlim)):
                limits = np.asarray(getter())
                setter(limits.mean() + (limits - limits.mean()) * factor)
            self._draw_idle()

    def _key_press(self, event):
        if event.key == "escape":
            self.cancelStroke()
            if self._panStart is not None:
                _, xr, yr = self._panStart
                self._set_canvas_limits(xr, yr)
            self._rotationStart = self._panStart = None
        elif event.key == "ctrl+z":
            self.undo()

    def close(self):
        child, curvature = self._canvasFigure, self.CurvatureFigure
        self._canvasFigure = self.CurvatureFigure = None
        if child is not None:
            plt.close(child)
        if curvature is not None:
            plt.close(curvature)
        plt.close(self.Figure)


def pipe_painter(model=None, options=None):
    return PipePainter(pipe_demo_model() if model is None else model, {} if options is None else options)
