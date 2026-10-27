import matplotlib.pyplot as plt
import numpy as np
import pytest
from matplotlib.backend_bases import KeyEvent, MouseEvent

from robot_safecontrol_moveit.pipe_assets.mat_io import load_mat, save_mat
from robot_safecontrol_moveit.pipe_assets.pipe_demo_model import pipe_demo_model
from robot_safecontrol_moveit.pipe_assets.pipe_painter import pipe_painter


def _event(canvas, name, x, y, **kwargs):
    event = MouseEvent(name, canvas, x, y, **kwargs)
    canvas.callbacks.process(name, event)


def _textbox_value(figure, label):
    for axes in figure.axes:
        if any(text.get_text() == label for text in axes.texts):
            values = [text.get_text() for text in axes.texts if text.get_text() != label]
            assert len(values) == 1
            return values[0]
    raise AssertionError(f"没有找到输入控件：{label}")


def _canvas_axes(app):
    return next(axes for axes in app.getCanvasFigure().axes if axes.get_xlabel().startswith("沿管位置"))


def _click_label(figure, label):
    figure.canvas.draw()
    artist = next(text for axes in figure.axes for text in axes.texts if text.get_text().startswith(label))
    bounds = artist.get_window_extent(figure.canvas.get_renderer())
    x, y = bounds.x0 + bounds.width / 2, bounds.y0 + bounds.height / 2
    _event(figure.canvas, "button_press_event", x, y, button=1)
    _event(figure.canvas, "button_release_event", x, y, button=1)


def _stroke(app, start, end):
    axes = _canvas_axes(app)
    canvas = axes.figure.canvas
    x, y = axes.transData.transform(start)
    _event(canvas, "button_press_event", x, y, button=1)
    x, y = axes.transData.transform(end)
    _event(canvas, "motion_notify_event", x, y, button=1)
    _event(canvas, "button_release_event", x, y, button=1)
    return axes.transData.inverted().transform([int(x), int(y)])


@pytest.fixture
def app(tmp_path):
    instance = pipe_painter(pipe_demo_model(), {"Visible": "off", "PathDirectory": tmp_path})
    yield instance
    instance.close()


def test_continuous_mouse_pan_uses_total_pixel_displacement(app):
    app.focusSegment(1)
    app.zoomCanvas(2)
    figure = app.getCanvasFigure()
    axes = _canvas_axes(app)
    figure.canvas.draw()
    before = np.asarray(axes.get_xlim())
    x, y = axes.transData.transform([before.mean(), np.mean(axes.get_ylim())])
    _event(figure.canvas, "button_press_event", x, y, button=1)
    _event(figure.canvas, "motion_notify_event", x + 20, y, button=1)
    _event(figure.canvas, "motion_notify_event", x + 40, y, button=1)
    _event(figure.canvas, "button_release_event", x + 40, y, button=1)
    expected = before - 40 / axes.bbox.width * np.ptp(before)
    np.testing.assert_allclose(axes.get_xlim(), expected, atol=1e-10)


def test_escape_during_mouse_pan_restores_original_view(app):
    app.focusSegment(1)
    app.zoomCanvas(2)
    figure = app.getCanvasFigure()
    axes = _canvas_axes(app)
    figure.canvas.draw()
    before_x, before_y = np.asarray(axes.get_xlim()), np.asarray(axes.get_ylim())
    x, y = axes.transData.transform([before_x.mean(), before_y.mean()])
    _event(figure.canvas, "button_press_event", x, y, button=1)
    _event(figure.canvas, "motion_notify_event", x + 20, y + 20, button=1)
    figure.canvas.callbacks.process("key_press_event", KeyEvent("key_press_event", figure.canvas, "escape"))
    np.testing.assert_allclose(axes.get_xlim(), before_x, atol=1e-10)
    np.testing.assert_allclose(axes.get_ylim(), before_y, atol=1e-10)


def test_reset_view_restores_model_extent_after_real_mouse_zoom(app):
    figure = app.Figure
    axes = next(axes for axes in figure.axes if axes.name == "3d")
    figure.canvas.draw()
    before = [axes.get_xlim(), axes.get_ylim(), axes.get_zlim()]
    x, y = axes.bbox.x0 + axes.bbox.width / 2, axes.bbox.y0 + axes.bbox.height / 2
    _event(figure.canvas, "scroll_event", x, y, step=2)
    assert not np.allclose([axes.get_xlim(), axes.get_ylim(), axes.get_zlim()], before)
    app.resetView()
    np.testing.assert_allclose([axes.get_xlim(), axes.get_ylim(), axes.get_zlim()], before, atol=1e-10)


def test_painter_rejects_three_column_keypoints_without_changing_path(app):
    app.setPath([[10, 0], [80, 90]])
    before = app.getState()["anchors"]
    with pytest.raises(ValueError):
        app.setPath([[10, 20, 30], [40, 50, 60]])
    np.testing.assert_array_equal(app.getState()["anchors"], before)


def test_loading_three_column_keypoints_rejects_file_without_changing_path(app, tmp_path):
    app.setPath([[10, 0], [30, 45], [50, 90]])
    source = app.export("valid-shape")[0]
    saved = load_mat(source)
    saved["trajectory"]["keypointsCanvas"] = saved["trajectory"]["keypointsCanvas"].reshape(2, 3)
    filename = save_mat(tmp_path / "three-column-keypoints.mat", saved)
    before = app.getState()["anchors"]
    with pytest.raises(ValueError):
        app.load(filename)
    np.testing.assert_array_equal(app.getState()["anchors"], before)


@pytest.mark.parametrize("update", ["initial", "setters", "load"])
def test_settings_controls_display_current_values(tmp_path, update):
    app = pipe_painter(pipe_demo_model(), {"Visible": "off", "PathDirectory": tmp_path,
                                          "ThetaRangeDeg": [-90, 270], "OffsetMM": 7,
                                          "SampleSpacingMM": 2})
    try:
        expected = {"offset": 7, "spacing": 2, "range": [-90, 270]}
        if update != "initial":
            app.setOffset(11)
            app.setSpacing(3)
            app.setThetaRange([-45, 225])
            expected = {"offset": 11, "spacing": 3, "range": [-45, 225]}
        if update == "load":
            app.setPath([[10, 0], [100, 90]])
            filename = app.export("settings")[0]
            app.setOffset(5)
            app.setSpacing(1)
            app.setThetaRange([-180, 540])
            app.load(filename)
        assert float(_textbox_value(app.Figure, "距离 mm ")) == expected["offset"]
        assert float(_textbox_value(app.Figure, "间隔 mm ")) == expected["spacing"]
        np.testing.assert_allclose(np.fromstring(_textbox_value(app.Figure, "范围 deg "), sep=","), expected["range"])
        if update == "setters":
            axes = _canvas_axes(app)
            assert len(axes.texts) == len(app.getState()["model"]["segments"])
            assert all(expected["range"][0] <= label.get_position()[1] <= expected["range"][1] for label in axes.texts)
    finally:
        app.close()


def test_curvature_display_clears_values_during_real_mouse_edit(app):
    app.setPath([[80, 0], [80, 90], [80, 180], [80, 270]])
    app.showCurvature()
    figure = app.getCanvasFigure()
    axes = _canvas_axes(app)
    figure.canvas.draw()
    x, y = axes.transData.transform([50, 10])
    _event(figure.canvas, "button_press_event", x, y, button=1)
    assert app.getState()["drawing"]
    lines = app.CurvatureFigure.axes[0].lines
    assert len(lines) == 1
    assert not np.any(np.isfinite(lines[0].get_ydata()))
    _event(figure.canvas, "button_release_event", x + 20, y + 20, button=1)
    assert not app.getState()["drawing"]
    curve = app.getCurvature()
    np.testing.assert_allclose(lines[0].get_ydata(), curve["kappa"], equal_nan=True)


def test_keypoint_drag_beyond_canvas_clamps_to_visible_boundary(app):
    app.setPath([[80, 30], [120, 60]])
    app.focusSegment(1)
    app.zoomCanvas(2)
    app.setViewAdjustment(False)
    app.setDrawMode(2)
    figure = app.getCanvasFigure()
    axes = _canvas_axes(app)
    figure.canvas.draw()
    x, y = axes.transData.transform([80, 30])
    _event(figure.canvas, "button_press_event", x, y, button=1)
    _event(figure.canvas, "motion_notify_event", axes.bbox.x1 + 20, y, button=1)
    _event(figure.canvas, "button_release_event", axes.bbox.x1 + 20, y, button=1)
    assert app.getState()["anchors"][0, 0] == pytest.approx(axes.get_xlim()[1])


def test_detached_controls_operate_all_drawing_modes_focus_and_docking(app):
    app.setPath([[10, 0], [20, 0]])
    app.setViewAdjustment(True)
    child = app.getCanvasFigure()
    assert app.getState()["viewAdjusting"]
    _click_label(child, "调整视图")
    assert not app.getState()["viewAdjusting"]

    _click_label(child, "点击添加")
    _stroke(app, [30, 10], [30, 10])
    np.testing.assert_allclose(app.getState()["anchors"], [[10, 0], [20, 0], [30, 10]], atol=1e-8)

    _click_label(app.Figure, "重新画线")
    _stroke(app, [40, 20], [80, 60])
    np.testing.assert_allclose(app.getState()["anchors"], [[40, 20], [80, 60]], atol=1e-8)

    _click_label(child, "从终点续画")
    _stroke(app, [80, 60], [120, 100])
    np.testing.assert_allclose(app.getState()["anchors"][[0, -1]], [[40, 20], [120, 100]], atol=1e-8)
    assert len(app.getState()["anchors"]) >= 3

    _click_label(child, "拖动关键点")
    destination = _stroke(app, [40, 20], [50, 30])
    np.testing.assert_allclose(app.getState()["anchors"][0], destination, atol=1e-8)

    focus = next(axes for axes in child.axes if any(text.get_text() == "管段序号 " for text in axes.texts))
    x, y = focus.bbox.x0 + focus.bbox.width / 2, focus.bbox.y0 + focus.bbox.height / 2
    _event(child.canvas, "button_press_event", x, y, button=1)
    _event(child.canvas, "button_release_event", x, y, button=1)
    for key in ("end", "backspace", "2", "enter"):
        child.canvas.callbacks.process("key_press_event", KeyEvent("key_press_event", child.canvas, key))
    state = app.getState()
    assert state["viewAdjusting"]
    assert 0 < state["canvasXLim"][0] < 180
    assert state["canvasXLim"][1] == pytest.approx(320)
    _click_label(child, "二维全览")
    np.testing.assert_allclose(app.getState()["canvasXLim"], [0, 320])
    _click_label(child, "收回画布")
    assert not app.getState()["canvasDetached"]
    assert not app.getState()["viewAdjusting"]
    assert app.getCanvasFigure() is app.Figure
    assert not plt.fignum_exists(child.number)
