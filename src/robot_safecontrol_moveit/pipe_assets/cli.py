import argparse
import hashlib
import importlib
import json
import platform
import time
from collections.abc import Mapping
from importlib.metadata import version
from pathlib import Path

import numpy as np

from .combined_pipe_model import combined_pipe_model
from .fit_pipe_path_nurbs import fit_pipe_path_nurbs
from .path_io import export_path, load_editable_path
from .pipe_complex_demo_model import pipe_complex_demo_model
from .pipe_demo_model import pipe_demo_model
from .pipe_examples import example_path
from .pipe_prepare_model import pipe_prepare_model
from .pipe_reference_path import pipe_reference_path

MODELS = {"demo": pipe_demo_model, "combined": combined_pipe_model, "complex": pipe_complex_demo_model}


def _model(args):
    if args.model_factory:
        module_name, function_name = args.model_factory.split(":", 1)
        factory = getattr(importlib.import_module(module_name), function_name)
        return pipe_prepare_model(factory())
    return pipe_prepare_model(MODELS[args.model]())


def _digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _json_value(value):
    if isinstance(value, np.ndarray):
        return _json_value(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    return value


def _write_manifest(output, command, summary, source, elapsed):
    output = Path(output)
    result = {
        "command": command, "length_unit": "mm", "angle_unit": "rad", "time_unit": "s",
        "python": platform.python_version(),
        "packages": {name: version(name) for name in ("numpy", "scipy", "matplotlib", "pymatreader", "h5py", "osqp")},
        "source_code_sha256": {path.name: _digest(path) for path in sorted(Path(__file__).parent.glob("*.py"))},
        "summary": summary, "elapsed_s": elapsed,
        "outputs": {str(path.relative_to(output)): _digest(path) for path in sorted(output.rglob("*")) if path.is_file()},
    }
    if source is not None:
        source = Path(source)
        result["input"] = {"path": str(source.resolve()), "sha256": _digest(source)}
    manifest = output / "manifest.json"
    manifest.write_text(json.dumps(_json_value(result), ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"output": str(output.resolve()), "manifest": str(manifest.resolve()), "summary": _json_value(summary)}, ensure_ascii=False, allow_nan=False))


def _new_output(path):
    path = Path(path)
    path.mkdir(parents=True, exist_ok=False)
    return path


def _existing_input(path):
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def _reference(args):
    model = _model(args)
    if args.input:
        saved = load_editable_path(model, _existing_input(args.input))
        points = saved["trajectory"]["keypointsCanvas"]
        definition = saved["trajectory"]["pathDefinition"]
        settings = dict(saved["settings"])
    else:
        points, definition = example_path(model)
        settings = {"OffsetMM": 5.0, "SampleSpacingMM": 1.0, "ThetaRangeDeg": [-180.0, 540.0]}
    if args.offset_mm is not None:
        settings["OffsetMM"] = args.offset_mm
    if args.spacing_mm is not None:
        settings["SampleSpacingMM"] = args.spacing_mm
    trajectory = pipe_reference_path(model, points, settings["OffsetMM"], settings["SampleSpacingMM"], definition)
    output = _new_output(args.output)
    export_path(model, trajectory, settings, output / "pipe_path")
    return {"points": len(trajectory["tcpPoints"]), "arc_mm": float(trajectory["arcLengthMM"][-1])}


def _fit(args):
    source = _existing_input(args.input)
    output = Path(args.output)
    if output.exists():
        raise FileExistsError(output)
    _, result = fit_pipe_path_nurbs(source, output, False)
    return {"points": len(result["data_points"]), "degree": result["degree"], "max_point_error_mm": result["max_point_error_mm"]}


def _plan(args):
    from .A_2_lixianjisuan import A_2_lixianjisuan

    fit_directory = Path(args.fit_dir)
    for name in ("control_points.txt", "knot_vector.txt"):
        _existing_input(fit_directory / name)
    output = _new_output(args.output)
    result = A_2_lixianjisuan(fit_directory, output)
    return {"plan_file": str(output / "nurbs_blocks.mat"), "fields": sorted(result)}


def _interpolate(args):
    from .A_3_shishichabu import A_3_shishichabu

    source = _existing_input(args.input)
    output = _new_output(args.output)
    result = A_3_shishichabu(source, output)
    return {"fields": sorted(result), "output": str(output)}


def _pose(args):
    from .A_4_zitai_gouzao import A_4_zitai_gouzao

    source = _existing_input(args.input)
    offline = _existing_input(args.offline)
    output = _new_output(args.output)
    result = A_4_zitai_gouzao({"show_figures": False}, source, output, Path(args.fit_dir), offline)
    return _pose_summary(result) | {"output": str(output)}


def _pose_summary(result):
    geometry = result["validation"]["geometry"]
    schedule = result["validation"]["schedule"]
    return {
        "passed": result["validation"]["passed"],
        "points": len(result["data"]["time"]),
        "duration_s": schedule["duration_s"],
        "time_stretch": schedule["time_stretch"],
        "max_axis_error_rad": geometry["fidelity"]["max_rad"],
        "omega_max_rad_s": schedule["angular"]["omega"]["measured_max"],
        "alpha_max_rad_s2": schedule["angular"]["alpha"]["measured_max"],
        "angular_limit_status": {
            name: schedule["angular"][name]["status"] for name in ("omega", "alpha")
        },
    }


def _run(args):
    from .A_2_lixianjisuan import A_2_lixianjisuan
    from .A_3_shishichabu import A_3_shishichabu

    source = _existing_input(args.input)
    if not args.skip_pose and source.suffix.lower() != ".mat":
        raise ValueError("完整姿态生成需要包含原始表面对应信息的 MAT 路径")
    output = _new_output(args.output)
    fit_directory = output / "fit"
    _, fit = fit_pipe_path_nurbs(source, fit_directory, False)
    A_2_lixianjisuan(fit_directory, output / "plan")
    A_3_shishichabu(output / "plan/nurbs_blocks.mat", output / "motion")
    result = {"input_points": len(fit["data_points"]), "max_point_error_mm": fit["max_point_error_mm"],
              "plan": str(output / "plan/nurbs_blocks.mat"), "motion": str(output / "motion")}
    if not args.skip_pose:
        from .A_4_zitai_gouzao import A_4_zitai_gouzao

        pose = A_4_zitai_gouzao({"show_figures": False}, source, output / "pose", fit_directory, output / "plan/nurbs_blocks.mat")
        result["pose"] = _pose_summary(pose)
    return result


def _painter(args):
    import matplotlib.pyplot as plt

    from .pipe_painter import pipe_painter

    source = _existing_input(args.input) if args.input else None
    app = pipe_painter(_model(args), {"PathDirectory": args.output, "Visible": "off" if args.hidden else "on"})
    if source is not None:
        app.load(source)
    if args.hidden:
        app.close()
    else:
        plt.show()
    return {"painter": "ready"}


def main(argv=None):
    parser = argparse.ArgumentParser(description="管道路径、NURBS、进给与姿态 Python 工具")
    commands = parser.add_subparsers(dest="command", required=True)
    handlers = {"reference": _reference, "fit": _fit, "plan": _plan, "interpolate": _interpolate,
                "pose": _pose, "run": _run, "painter": _painter}
    for name, handler in handlers.items():
        command = commands.add_parser(name)
        command.set_defaults(handler=handler)
        command.add_argument("--output", type=Path, required=True)
        if name in ("reference", "fit", "interpolate", "pose", "run", "painter"):
            command.add_argument("--input", type=Path, required=name in ("fit", "interpolate", "pose", "run"))
        if name in ("reference", "painter"):
            command.add_argument("--model", choices=MODELS, default="demo")
            command.add_argument("--model-factory", help="自定义模型的 MODULE:FUNCTION")
        if name in ("plan", "pose"):
            command.add_argument("--fit-dir", type=Path, required=True)
        if name == "pose":
            command.add_argument("--offline", type=Path, required=True)
        if name == "reference":
            command.add_argument("--offset-mm", type=float)
            command.add_argument("--spacing-mm", type=float)
        if name == "run":
            command.add_argument("--skip-pose", action="store_true")
        if name == "painter":
            command.add_argument("--hidden", action="store_true")
    args = parser.parse_args(argv)
    started = time.perf_counter()
    result = args.handler(args)
    if args.command != "painter":
        _write_manifest(args.output, args.command, result, getattr(args, "input", None), time.perf_counter() - started)
    return 0
