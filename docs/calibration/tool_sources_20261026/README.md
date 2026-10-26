# 本次标定工具

这些文件是本次实际运行的工具源码。使用时恢复到仓库根目录的 `.scratch/lidar-camera-calibration-20261026/`，数据目录和结果目录保留 `sceneXX`、`sceneXX-result` 的名称。

`solve_scene.sh` 使用相邻 `.scratch/lidar-camera-calibration/fast-calib-build/fast_calib` 原生程序；`scene02.yaml` 保存本次内参、板尺寸和雷达处理范围，运行时通过参数指定实际场景路径。Python 依赖和 FAST-Calib 来源见 [标定记录](../lidar_camera_20261026.md)。

`inspect_scene.py` 使用 ROS 2 Humble 的系统 Python。`check_camera.py` 使用 OpenCV。`check_measurements.py`、`evaluate_calibration.py`、`apply_calibration.py`、`plot_calibration.py`、`verify_artifacts.py` 使用已有 venv 中的 Open3D、SciPy、NumPy、OpenCV 和 Matplotlib。绘图字体使用本机 SimSun 与 Times New Roman。
