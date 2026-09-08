"""Joint time-series recording and plotting helpers."""

from __future__ import annotations

import csv
import os
from pathlib import Path
from typing import Iterable, List, Sequence

import numpy as np


class JointTrajectoryRecorder:
    """Record joint position/velocity samples and export CSV/PNG plots.

    ``stream_path`` keeps long simulations bounded in memory.  Raw samples are
    hydrated only after control has finished, when CSV/PNG artifacts are made.
    """

    def __init__(self, joint_names: Sequence[str], *,
                 stream_path: str | os.PathLike | None = None,
                 stream_flush_rows: int = 128) -> None:
        self.joint_names = list(joint_names)
        if not self.joint_names:
            raise ValueError("joint_names must not be empty")
        if int(stream_flush_rows) < 1:
            raise ValueError("stream_flush_rows must be >= 1")
        self._rows: List[dict] = []
        self._last_time_s: float | None = None
        self._stream_path = Path(stream_path) if stream_path else None
        self._stream_flush_rows = int(stream_flush_rows)
        self._stream_row_count = 0
        self._stream_handle = None
        self._stream_writer = None
        if self._stream_path is not None:
            self._stream_path.parent.mkdir(parents=True, exist_ok=True)
            self._stream_handle = self._stream_path.open(
                "w", newline="", encoding="utf-8", buffering=1024 * 1024)
            self._stream_writer = csv.DictWriter(
                self._stream_handle, fieldnames=self._raw_fieldnames())
            self._stream_writer.writeheader()
            self._stream_handle.flush()

    def _raw_fieldnames(self) -> list[str]:
        fields = ["time_s", "phase"]
        for prefix in ("q", "dq"):
            fields.extend(f"{prefix}{index}" for index in range(len(self.joint_names)))
        return fields

    @property
    def row_count(self) -> int:
        return self._stream_row_count if self._stream_path is not None else len(self._rows)

    @property
    def rows(self) -> Iterable[dict]:
        self._hydrate_stream_rows()
        return tuple(self._rows)

    def record(self, time_s: float, phase: str, q, dq) -> None:
        if self._stream_path is not None and self._rows:
            raise RuntimeError("cannot append after streaming joint samples were hydrated")
        q_arr = np.asarray(q, dtype=float).reshape(len(self.joint_names))
        dq_arr = np.asarray(dq, dtype=float).reshape(len(self.joint_names))
        t = float(time_s)
        previous_time = self._last_time_s
        if previous_time is None and self._rows:
            previous_time = float(self._rows[-1]["time_s"])
        if previous_time is not None and t <= previous_time:
            t = previous_time + 1.0e-9
        self._last_time_s = t
        if self._stream_path is not None:
            if self._stream_writer is None:
                raise RuntimeError("streaming joint recorder has already been finalized")
            row = {"time_s": t, "phase": str(phase)}
            for index in range(len(self.joint_names)):
                row[f"q{index}"] = float(q_arr[index])
                row[f"dq{index}"] = float(dq_arr[index])
            self._stream_writer.writerow(row)
            self._stream_row_count += 1
            if self._stream_row_count % self._stream_flush_rows == 0:
                self._stream_handle.flush()
            return
        self._rows.append({
            "time_s": t,
            "phase": str(phase),
            "q": q_arr.copy(),
            "dq": dq_arr.copy(),
        })

    def _finalize_stream(self) -> None:
        if self._stream_handle is not None:
            self._stream_handle.flush()
            self._stream_handle.close()
            self._stream_handle = None
            self._stream_writer = None

    def _hydrate_stream_rows(self) -> None:
        if self._stream_path is None or self._rows or self._stream_row_count == 0:
            return
        self._finalize_stream()
        with self._stream_path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            for row in reader:
                self._rows.append({
                    "time_s": float(row["time_s"]),
                    "phase": str(row["phase"]),
                    "q": np.asarray([
                        float(row[f"q{index}"])
                        for index in range(len(self.joint_names))], dtype=float),
                    "dq": np.asarray([
                        float(row[f"dq{index}"])
                        for index in range(len(self.joint_names))], dtype=float),
                })

    def cleanup_stream_artifact(self) -> None:
        """Remove the raw spool only after final workflow artifacts succeeded."""
        self._finalize_stream()
        if self._stream_path is not None:
            self._stream_path.unlink(missing_ok=True)
            self._stream_path = None
            self._stream_row_count = 0
            self._rows.clear()

    def _arrays(self):
        self._hydrate_stream_rows()
        n = len(self.joint_names)
        if not self._rows:
            return np.zeros(0), np.empty((0, n)), np.empty((0, n)), np.empty((0, n))

        times = np.asarray([float(row["time_s"]) for row in self._rows], dtype=float)
        q = np.vstack([row["q"] for row in self._rows])
        dq = np.vstack([row["dq"] for row in self._rows])
        ddq = _finite_difference(times, dq)
        return times, q, dq, ddq

    def display_arrays(self):
        """Return arrays converted to plot-friendly units.

        J1 is prismatic and shown in mm.  J2-J9 are revolute and shown in deg.
        """
        times, q, dq, ddq = self._arrays()
        q_disp = q.copy()
        dq_disp = dq.copy()
        ddq_disp = ddq.copy()
        if q_disp.shape[1] >= 1:
            q_disp[:, 0] *= 1000.0
            dq_disp[:, 0] *= 1000.0
            ddq_disp[:, 0] *= 1000.0
        if q_disp.shape[1] > 1:
            scale = 180.0 / np.pi
            q_disp[:, 1:] *= scale
            dq_disp[:, 1:] *= scale
            ddq_disp[:, 1:] *= scale
        return times, q_disp, dq_disp, ddq_disp

    def plot_display_arrays(self):
        """Return display-unit arrays for PNG plots.

        The CSV keeps raw per-cycle command velocities.  PNG plots use a
        light phase-aware smoothing pass for velocity and acceleration so
        visual inspection is not dominated by single-cycle QP/phase spikes.
        Smoothing never crosses phase boundaries.
        """
        times, q_disp, dq_disp, _ddq_disp = self.display_arrays()
        phases = [row["phase"] for row in self._rows]
        dq_plot = _phasewise_smooth(times, dq_disp, phases, window_s=0.08)
        ddq_plot = _finite_difference(times, dq_plot)
        ddq_plot = _phasewise_smooth(times, ddq_plot, phases, window_s=0.12)
        return times, q_disp, dq_plot, ddq_plot

    def write_csv(self, path: str | os.PathLike) -> Path:
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        times, q, dq, ddq = self._arrays()

        fields = ["time_s", "phase"]
        for prefix in ("q", "dq", "ddq"):
            fields.extend(f"{prefix}{i}" for i in range(len(self.joint_names)))

        with out.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=fields)
            writer.writeheader()
            for idx, row in enumerate(self._rows):
                csv_row = {
                    "time_s": float(times[idx]),
                    "phase": row["phase"],
                }
                for j in range(len(self.joint_names)):
                    csv_row[f"q{j}"] = float(q[idx, j])
                    csv_row[f"dq{j}"] = float(dq[idx, j])
                    csv_row[f"ddq{j}"] = float(ddq[idx, j])
                writer.writerow(csv_row)
        return out

    def write_plots(self, output_dir: str | os.PathLike, prefix: str = "joint_workflow") -> List[Path]:
        out_dir = Path(output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        times, q, dq, ddq = self.plot_display_arrays()
        if len(times) == 0:
            return []

        import matplotlib
        matplotlib.use("Agg", force=True)
        import matplotlib.pyplot as plt

        outputs = [
            self._write_grid_plot(
                plt, out_dir / f"{prefix}_positions.png", times, q,
                "Joint positions over complete workflow",
                ("mm", "deg"),
            ),
            self._write_grid_plot(
                plt, out_dir / f"{prefix}_velocities.png", times, dq,
                "Joint velocities over complete workflow",
                ("mm/s", "deg/s"),
            ),
            self._write_grid_plot(
                plt, out_dir / f"{prefix}_accelerations.png", times, ddq,
                "Joint accelerations over complete workflow",
                ("mm/s^2", "deg/s^2"),
            ),
            self._write_overview_plot(
                plt, out_dir / f"{prefix}_overview.png", times, q, dq, ddq),
        ]
        return outputs

    def _write_grid_plot(self, plt, path: Path, times, values, title: str, units) -> Path:
        n = len(self.joint_names)
        n_cols = 3 if n > 2 else n
        n_rows = int(np.ceil(n / n_cols))
        fig, axes = plt.subplots(
            n_rows, n_cols,
            figsize=(4.2 * n_cols, 2.45 * n_rows),
            sharex=True,
            squeeze=False,
        )
        phase_boundaries = self._phase_boundaries()

        for idx, name in enumerate(self.joint_names):
            ax = axes[idx // n_cols][idx % n_cols]
            ax.plot(times, values[:, idx], linewidth=1.2)
            for boundary_t, _phase in phase_boundaries:
                ax.axvline(boundary_t, color="0.72", linewidth=0.7, linestyle="--")
            unit = units[0] if idx == 0 else units[1]
            ax.set_title(name)
            ax.set_ylabel(unit)
            ax.grid(True, alpha=0.3)

        for idx in range(n, n_rows * n_cols):
            axes[idx // n_cols][idx % n_cols].axis("off")
        for ax in axes[-1]:
            ax.set_xlabel("workflow time (s)")
        fig.suptitle(title)
        fig.tight_layout(rect=[0.0, 0.0, 1.0, 0.96])
        fig.savefig(path, dpi=150)
        plt.close(fig)
        return path

    def _write_overview_plot(self, plt, path: Path, times, q, dq, ddq) -> Path:
        fig, axes = plt.subplots(3, 1, figsize=(14.0, 10.5), sharex=True)
        specs = [
            (q, "Position", "revolute joints (deg)", "J1 (mm)"),
            (dq, "Velocity", "revolute joints (deg/s)", "J1 (mm/s)"),
            (ddq, "Acceleration", "revolute joints (deg/s^2)", "J1 (mm/s^2)"),
        ]
        phase_boundaries = self._phase_boundaries()

        for ax, (values, title, left_label, right_label) in zip(axes, specs):
            if values.shape[1] > 1:
                for idx, name in enumerate(self.joint_names[1:], start=1):
                    ax.plot(times, values[:, idx], linewidth=1.0, label=name)
            ax.set_ylabel(left_label)
            ax.set_title(title)
            ax.grid(True, alpha=0.3)
            for boundary_t, _phase in phase_boundaries:
                ax.axvline(boundary_t, color="0.72", linewidth=0.8, linestyle="--")

            ax_right = ax.twinx()
            ax_right.plot(times, values[:, 0], color="black", linewidth=1.4,
                          linestyle="--", label=self.joint_names[0])
            ax_right.set_ylabel(right_label)

            lines_left, labels_left = ax.get_legend_handles_labels()
            lines_right, labels_right = ax_right.get_legend_handles_labels()
            ax.legend(lines_left + lines_right, labels_left + labels_right,
                      loc="upper right", ncol=5, fontsize=8)

        axes[-1].set_xlabel("complete workflow time (s)")
        fig.suptitle("All joints over complete workflow")
        fig.tight_layout(rect=[0.0, 0.0, 1.0, 0.96])
        fig.savefig(path, dpi=150)
        plt.close(fig)
        return path

    def _phase_boundaries(self):
        boundaries = []
        prev_phase = None
        for row in self._rows:
            phase = row["phase"]
            if prev_phase is not None and phase != prev_phase:
                boundaries.append((float(row["time_s"]), phase))
            prev_phase = phase
        return boundaries


def _finite_difference(times, values):
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return values.copy()
    if len(times) < 2:
        return np.zeros_like(values)

    safe_times = times.copy()
    for idx in range(1, len(safe_times)):
        if safe_times[idx] <= safe_times[idx - 1]:
            safe_times[idx] = safe_times[idx - 1] + 1.0e-9
    return np.gradient(values, safe_times, axis=0, edge_order=1)


def _phasewise_smooth(times, values, phases, window_s: float):
    times = np.asarray(times, dtype=float)
    values = np.asarray(values, dtype=float)
    if values.size == 0 or len(times) < 5:
        return values.copy()

    smoothed = values.copy()
    phases = list(phases)
    start = 0
    while start < len(times):
        phase = phases[start]
        end = start + 1
        while end < len(times) and phases[end] == phase:
            end += 1

        count = end - start
        if count >= 5:
            local_t = times[start:end]
            dts = np.diff(local_t)
            positive_dts = dts[dts > 1.0e-9]
            median_dt = float(np.median(positive_dts)) if len(positive_dts) else 0.0
            if median_dt > 0.0:
                window = int(round(window_s / median_dt))
                window = max(3, min(31, window))
                if window % 2 == 0:
                    window += 1
                if window <= count:
                    pad = window // 2
                    kernel = np.ones(window, dtype=float) / float(window)
                    padded = np.pad(values[start:end], ((pad, pad), (0, 0)), mode="edge")
                    for joint_idx in range(values.shape[1]):
                        smoothed[start:end, joint_idx] = np.convolve(
                            padded[:, joint_idx], kernel, mode="valid")
        start = end
    return smoothed


def make_joint_recorder(output_dir, joint_names):
    if not output_dir:
        return None
    raw_path = Path(output_dir) / ".joint_workflow_raw_samples.csv"
    return JointTrajectoryRecorder(joint_names, stream_path=raw_path)


def record_runner_joint_sample(runner, phase, dq=None, dt_increment=0.0) -> None:
    recorder = getattr(runner, "joint_recorder", None)
    if recorder is None:
        return
    runner._joint_workflow_time_s += max(float(dt_increment), 0.0)
    if dq is None:
        dq = np.zeros(len(recorder.joint_names))
    recorder.record(runner._joint_workflow_time_s, phase, runner.q.copy(), dq)


def write_joint_workflow_outputs(recorder, output_dir):
    out_dir = Path(output_dir)
    csv_path = recorder.write_csv(out_dir / "joint_workflow_timeseries.csv")
    png_paths = recorder.write_plots(out_dir, prefix="joint_workflow")
    recorder.cleanup_stream_artifact()
    return csv_path, png_paths
