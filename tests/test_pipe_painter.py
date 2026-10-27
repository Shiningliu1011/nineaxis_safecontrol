from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pytest

from robot_safecontrol_moveit.pipe_assets.mat_io import load_mat
from robot_safecontrol_moveit.pipe_assets.pipe_demo_model import pipe_demo_model
from robot_safecontrol_moveit.pipe_assets.pipe_painter import pipe_painter


def test_painter_exports_and_reloads_editable_path(tmp_path):
    app = pipe_painter(pipe_demo_model(), {"Visible": "off", "PathDirectory": tmp_path})
    anchors = np.array([[10.0, 0.0], [50.0, 0.0], [100.0, 0.0]])
    try:
        app.setPath(anchors)
        sample = app.getSamples()
        np.testing.assert_allclose(sample["tcpPoints"][[0, -1]], [[40, 0, 10], [40, 0, 100]])
        files = app.export("reference")
        assert [Path(path).suffix for path in files] == [".mat", ".csv"]
        saved = load_mat(files[0])
        np.testing.assert_allclose(saved["trajectory"]["tcpPoints"], sample["tcpPoints"])
        app.setPath([[20.0, 90.0], [80.0, 90.0]])
        app.load(files[0])
        np.testing.assert_array_equal(app.getState()["anchors"], anchors)
        app.setOffset(10)
        np.testing.assert_allclose(app.getSamples()["tcpPoints"][[0, -1]], [[45, 0, 10], [45, 0, 100]])
    finally:
        app.close()


def test_painter_strokes_undo_and_view_modes(tmp_path):
    app = pipe_painter(pipe_demo_model(), {"Visible": "off", "PathDirectory": tmp_path})
    try:
        app.setPath([[10, 0], [80, 0]])
        app.beginStroke([80, 0], True)
        app.appendStroke([120, 20])
        app.cancelStroke()
        np.testing.assert_array_equal(app.getState()["anchors"], [[10, 0], [80, 0]])
        app.moveKeypoint(2, [90, 10])
        app.undo()
        np.testing.assert_array_equal(app.getState()["anchors"], [[10, 0], [80, 0]])
        app.focusSegment(2)
        assert app.getState()["canvasDetached"]
        assert app.getState()["viewAdjusting"]
        with pytest.raises(ValueError):
            app.beginStroke([90, 10], False)
        app.zoomCanvas(2)
        app.panCanvas([-0.1, 0])
        app.dockCanvas()
        assert not app.getState()["canvasDetached"]
        assert not app.getState()["viewAdjusting"]
        app.setLayer("real", False)
        assert not app.getState()["layers"][0]
    finally:
        app.close()


def test_curvature_window_reuses_current_geometry_and_cleans_up(tmp_path):
    app = pipe_painter(pipe_demo_model(), {"Visible": "off", "PathDirectory": tmp_path})
    try:
        app.setPath([[80, 0], [80, 90], [80, 180], [80, 270]])
        app.showCurvature()
        first_window = app.CurvatureFigure
        curve = app.getCurvature()
        np.testing.assert_allclose(curve["kappa"][curve["valid"]], 1 / 40, atol=2e-5)
        app.setOffset(10)
        app.showCurvature()
        assert app.CurvatureFigure is first_window
        changed = app.getCurvature()
        np.testing.assert_allclose(changed["kappa"][changed["valid"]], 1 / 45, atol=2e-5)
        assert app.getState()["curvatureVisible"]
        plt.close(first_window)
        assert not app.getState()["curvatureVisible"]
        app.showCurvature()
        reopened = app.CurvatureFigure.number
    finally:
        app.close()
    assert not plt.fignum_exists(reopened)
