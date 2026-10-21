from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
import math
import shutil

from .static_experiment_capacity import RecordingBudgetExhausted


MAXIMUM_RECORD_NAME_BYTES = 256


@dataclass(frozen=True)
class RecordingStorageLimits:
    database_bytes: int
    wal_bytes: int
    other_bytes: int
    minimum_free_bytes: int
    completion_bytes: int
    record_bytes: dict[str, int] | None = None
    record_write_bytes: dict[str, int] | None = None
    completion_base_bytes: int = 0
    pending_cycle_bytes: int = 0
    statistics_bytes: int = 65536

    def __post_init__(self):
        if any(type(getattr(self, key)) is not int or getattr(self, key) <= 0 for key in (
                "database_bytes", "wal_bytes", "other_bytes", "minimum_free_bytes", "completion_bytes",
                "statistics_bytes")):
            raise ValueError("记录空间额度必须为正整数")
        if any(type(value) is not int or value < 0 for value in (
                self.completion_base_bytes, self.pending_cycle_bytes)):
            raise ValueError("任务收尾额度必须为非负整数")
        for sizes in (self.record_bytes, self.record_write_bytes):
            if sizes is not None and (
                    not sizes or any(type(value) is not int or value <= 0 for value in sizes.values())):
                raise ValueError("逐流记录尺寸必须为正整数")
        if self.completion_bytes > self.minimum_free_bytes:
            raise ValueError("收尾额度超过文件系统保留空间")
        if self.completion_base_bytes > self.completion_bytes:
            raise ValueError("收尾空间不足以保存已开始任务")


class RecordingStorageGuard:
    def __init__(self, path: Path, connection, limits: RecordingStorageLimits):
        self.path, self.connection, self.limits = path, connection, limits
        self.reason = None
        self.last_admitted = self.measure()
        self.trigger = None
        self.maximum_record_bytes = {}
        self.maximum_write_bytes = {}
        self.last_wal_bytes = self.last_admitted["wal_bytes"]
        self.maximum_pending_cycles = 0

    def measure(self) -> dict:
        page_size = self.connection.execute("PRAGMA page_size").fetchone()[0]
        database_bytes = self.connection.execute("PRAGMA page_count").fetchone()[0] * page_size
        wal = Path(str(self.path) + "-wal")
        excluded = {
            f"{role}.sqlite3{suffix}"
            for role in ("plant", "controller") for suffix in ("", "-wal", "-shm")
        } | {self.path.name, wal.name, self.path.name + "-shm"}
        other_bytes = sum(
            item.stat().st_blocks * 512 for item in self.path.parent.rglob("*")
            if not item.name.endswith(".pending") and item.is_file() and item.name not in excluded
        )
        return {
            "database_bytes": database_bytes,
            "wal_bytes": wal.stat().st_blocks * 512 if wal.exists() else 0,
            "other_bytes": other_bytes,
            "free_bytes": shutil.disk_usage(self.path.parent).free,
        }

    def check(self, *, pending_cycles: int = 0, pending_record: tuple[str, int] | None = None,
              pending_statistics: bool = False) -> None:
        if type(pending_cycles) is not int or pending_cycles < 0:
            raise ValueError("待收尾周期数量必须为非负整数")
        observed = self.measure()
        pending_bytes = 0
        if pending_record is not None:
            stream, raw_bytes = pending_record
            if self.limits.record_bytes is not None and (
                    stream not in self.limits.record_bytes or raw_bytes > self.limits.record_bytes[stream]):
                self.reason = self.reason or f"record_bytes/{stream}"
            if self.limits.record_write_bytes is not None:
                pending_bytes = self.limits.record_write_bytes[stream]
        if self.reason is None:
            self.reason = next(
                (key for key in ("database_bytes", "wal_bytes", "other_bytes")
                 if observed[key] + (pending_bytes if key != "other_bytes" else
                                     self.limits.statistics_bytes if pending_statistics else 0)
                 > getattr(self.limits, key)),
                "free_bytes" if observed["free_bytes"] < self.limits.minimum_free_bytes else None,
            )
            if self.reason is not None:
                self.trigger = observed
        if self.reason is None and (
                self.limits.completion_base_bytes + pending_cycles * self.limits.pending_cycle_bytes
                > self.limits.completion_bytes):
            self.reason, self.trigger = "completion_bytes", observed
        if self.reason is not None:
            if self.trigger is None:
                self.trigger = observed
            raise RecordingBudgetExhausted(f"budget_exhausted: storage/{self.reason}")
        self.maximum_pending_cycles = max(self.maximum_pending_cycles, pending_cycles)
        self.last_admitted = observed

    def check_statistics_allocation(self, allocated_bytes: int, *, pending: bool) -> None:
        if not 0 <= allocated_bytes <= self.limits.statistics_bytes:
            raise ValueError("统计文件超过已配置字节上限")
        snapshot = self.snapshot()
        extra = allocated_bytes if pending else 0
        if snapshot["completion_growth_bytes"] + extra > self.limits.completion_bytes:
            raise ValueError("统计文件与已开始任务超过收尾保留空间")
        if snapshot["observed"]["free_bytes"] < extra:
            raise ValueError("文件系统空间不足以保存统计文件")

    def record_written(self, stream: str, raw_bytes: int) -> None:
        wal_bytes = Path(str(self.path) + "-wal").stat().st_blocks * 512
        growth = max(0, wal_bytes - self.last_wal_bytes)
        self.last_wal_bytes = wal_bytes
        self.maximum_record_bytes[stream] = max(self.maximum_record_bytes.get(stream, 0), raw_bytes)
        self.maximum_write_bytes[stream] = max(self.maximum_write_bytes.get(stream, 0), growth)
        if self.limits.record_bytes is not None and (
                stream not in self.limits.record_bytes or raw_bytes > self.limits.record_bytes[stream]):
            self.reason = self.reason or f"record_bytes/{stream}"
        if self.limits.record_write_bytes is not None and (
                stream not in self.limits.record_write_bytes
                or growth > self.limits.record_write_bytes[stream]):
            self.reason = self.reason or f"record_write_bytes/{stream}"

    def snapshot(self) -> dict:
        observed = self.measure()
        # 数据库页数包括 checkpoint 后的完整内容，收尾增加量同时核对 WAL。
        completion = sum(max(0, observed[name] - self.last_admitted[name]) for name in (
            "database_bytes", "wal_bytes", "other_bytes",
        ))
        return {
            "limits": asdict(self.limits), "reason": self.reason,
            "last_admitted": self.last_admitted, "trigger": self.trigger, "observed": observed,
            "completion_growth_bytes": completion,
            "completion_within_reserve": completion <= self.limits.completion_bytes,
            "maximum_record_bytes": dict(self.maximum_record_bytes),
            "maximum_write_bytes": dict(self.maximum_write_bytes),
            "maximum_pending_cycles": self.maximum_pending_cycles,
        }


def schema_record_bounds(pairs: int) -> dict:
    if type(pairs) is not int or pairs < 0:
        raise ValueError("几何组合数量必须为非负整数")
    rows = 36 + pairs
    qp_shapes = {
        "P": [10, 10], "c": [10], "G": [rows + 2, 10], "h": [rows + 2],
        "variable_scale": [10], "solution": [10], "dual": [rows + 2], "velocity": [9],
    }
    sns_shapes = {
        "J": [5, 9], "desired": [5], "G": [rows, 9], "h": [rows], "lower": [rows],
        "nominal_feed": [1], "solution": [9], "velocity": [9], "saturated": [rows],
    }
    numeric_slots = max(sum(math.prod(shape) for shape in shapes.values())
                        for shapes in (qp_shapes, sns_shapes)) + 64
    name_bytes = MAXIMUM_RECORD_NAME_BYTES
    # MessagePack float64 和有符号/无符号 64 位整数最多占用九个字节。
    # 固定字段名、certificate、来源映射和容器头单独计入，数组长度取生产 shape。
    trial_bytes = numeric_slots * 9 + 4096 + 2 * name_bytes + 5 * (rows + 66)
    geometry_bytes = pairs * (18 * 9 + 1024 + 2 * name_bytes)
    candidate = math.ceil((9 * trial_bytes + geometry_bytes + 4096 + 64 * 9) / 4096) * 4096
    return {
        "maximum_trials": 9, "maximum_name_utf8_bytes": name_bytes,
        "qp_shapes": qp_shapes, "sns_shapes": sns_shapes, "numeric_slots_per_trial": numeric_slots,
        "trial_bytes": trial_bytes, "geometry_bytes": geometry_bytes,
        "candidate_bytes": candidate,
        "solver_failure_bytes": math.ceil((numeric_slots * 9 + 65536 + 5 * (rows + 66)) / 4096) * 4096,
        "other_record_bytes": 65536,
    }


def runtime_storage_configuration(model: dict, wal_bytes_per_record: dict, volume_budget: dict) -> dict:
    databases = model["databases"]
    checkpoint_bytes = sum(
        math.ceil(row["database_budget_bytes"] / 4096) * 4096 for row in databases.values()
    )
    if checkpoint_bytes + model["other_retained_bytes"] > model["retained_run_bytes"]:
        raise ValueError("逐 writer 数据库与其他保留文件超过单次记录额度")
    # 两个 worker 各自保存触发记录和全部在途任务，checkpoint 的完整数据库空间单列。
    completion_bytes = (volume_budget["reserve_bytes"] - checkpoint_bytes
                        - model["other_retained_bytes"]) // 2
    counts = {
        "plant": {"feedback": 1, "recording_delay": 1, "received_command": 1, "admission": 2, "event": 1},
        "controller": {
            "candidate": 1, "cycle": 1, "command": 1, "admission": 1, "rejection": 1,
            "transition": 4, "qp_failure": 1, "sns_failure": 1, "event": 1,
        },
    }
    writers, proofs = {}, {}
    schema = schema_record_bounds(model["maximum_geometry_pairs"])
    for role, database in databases.items():
        raw = {
            name.split("/", 1)[1]: (
                schema["candidate_bytes"] if name == "controller/candidate" else
                schema["solver_failure_bytes"] if name.endswith(("/qp_failure", "/sns_failure")) else
                schema["other_record_bytes"]
            )
            for name, item in model["streams"].items() if name.startswith(role + "/")
        }
        if any(item["maximum_raw_bytes"] > raw[name.split("/", 1)[1]]
               for name, item in model["streams"].items() if name.startswith(role + "/")):
            raise ValueError("已审计记录超过生产 schema 的收尾尺寸界限")
        # 两棵 B-tree 的路径与相邻页，以及 payload overflow pages 均保留写入空间。
        tree_pages = 8 * (math.ceil(math.log2(database["maximum_records"] + 1)) + 1) + 8
        writes = {
            name: (tree_pages + math.ceil((2 * size + 4096) / 4096)) * (2 * 4096 + 24)
            for name, size in raw.items()
        }
        statistics_bytes = model["statistics_bytes"][role]
        base = sum(writes[name] * count for name, count in counts[role].items()) + statistics_bytes
        limits = RecordingStorageLimits(
            database_bytes=math.ceil(database["database_budget_bytes"] / 4096) * 4096,
            wal_bytes=math.ceil(wal_bytes_per_record[role] * database["maximum_records"] * 1.25 / 4096) * 4096,
            other_bytes=max(model["other_retained_bytes"], 4096),
            minimum_free_bytes=volume_budget["reserve_bytes"],
            completion_bytes=completion_bytes, record_bytes=raw, record_write_bytes=writes,
            completion_base_bytes=base,
            pending_cycle_bytes=writes.get("cycle", 0),
            statistics_bytes=statistics_bytes,
        )
        writers[role] = asdict(limits)
        proofs[role] = {
            "completion_record_counts": counts[role], "record_write_allowance_bytes": writes,
            "btree_changed_page_allowance": tree_pages,
            "statistics_bytes": statistics_bytes,
        }
    if sum(row["wal_bytes"] for row in writers.values()) > volume_budget["transient_run_bytes"]:
        raise ValueError("两个 writer 的 WAL 额度超过单次工作空间")
    return {
        "writers": writers, "completion_proof": proofs,
        "checkpoint_bytes": checkpoint_bytes, "reserve_bytes": volume_budget["reserve_bytes"],
        "retained_run_bytes": model["retained_run_bytes"],
        "schema_bounds": schema,
        "encoding_scope": "validated_schema_and_measured_envelope",
    }
