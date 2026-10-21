import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.ft2font import FT2Font
from matplotlib.text import Text
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "docs/proposal-static-obstacles/data"
OUTPUT = ROOT / "picture/proposal-static-obstacles/summary"


def checksum(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    fonts, coverage = {}, {}
    for family in ("SimSun", "Times New Roman"):
        path = Path(font_manager.findfont(font_manager.FontProperties(family=family), fallback_to_default=False))
        face = FT2Font(str(path))
        if face.family_name != family:
            raise ValueError(f"字体身份不一致：{family}")
        fonts[family] = {"family": face.family_name, "sha256": checksum(path)}
        coverage[family] = set(face.get_charmap())
    matplotlib.rcParams.update({"font.family": ["Times New Roman", "SimSun"], "font.size": 12, "svg.fonttype": "none", "pdf.fonttype": 42, "axes.unicode_minus": True})
    with (DATA / "metrics.csv").open(encoding="utf-8", newline="") as stream:
        rows = {row["run"]: row for row in csv.DictReader(stream)}
    OUTPUT.mkdir(parents=True, exist_ok=True)
    exported = {}

    def save(figure, name):
        for text in figure.findobj(match=Text):
            for character in text.get_text():
                family = "SimSun" if "\u3000" <= character <= "\uffff" else "Times New Roman"
                if not character.isspace() and ord(character) not in coverage[family]:
                    raise ValueError(f"字体缺少字符：{character}")
        for suffix in ("png", "svg", "pdf"):
            path = OUTPUT / f"{name}.{suffix}"
            figure.savefig(path, dpi=220, bbox_inches="tight")
            exported[path.name] = {"sha256": checksum(path), "bytes": path.stat().st_size}
            if suffix == "png":
                with Image.open(path) as image:
                    if image.width < 1000 or image.height < 600:
                        raise ValueError("图片尺寸不足")
                    image.verify()
        plt.close(figure)

    figure, axis = plt.subplots(figsize=(9, 5), layout="constrained")
    names = ["s0", "s4"]
    for offset, key, label, color in ((-.24, "path_rms_mm", "RMS", "#286a8e"), (0, "path_p95_mm", "P95", "#3b9c87"), (.24, "path_max_mm", "最大值", "#c87937")):
        values = [float(rows[name][key]) for name in names]
        bars = axis.bar([index + offset for index in range(2)], values, width=.23, label=label, color=color)
        axis.bar_label(bars, fmt="%.4f", padding=3, fontsize=10)
    axis.set_xticks([0, 1], ["S0 / D", "S4 / D"])
    axis.set_ylabel("路径距离 (mm)")
    axis.set_title("完整路径仿真样本的跟踪误差")
    axis.set_ylim(0, max(float(rows[name]["path_max_mm"]) for name in names) * 1.4)
    axis.legend()
    axis.grid(axis="y", alpha=.2)
    save(figure, "tracking-summary")

    figure, axis = plt.subplots(figsize=(10, 5), layout="constrained")
    names = ["s0", "s4", "s5-ext4", "s5-ntfs", "s5-windows"]
    values = [float(rows[name]["cycle_p999_ms"]) for name in names]
    bars = axis.bar(range(5), values, color=["#286a8e" if value <= 80 else "#bc533d" for value in values])
    axis.bar_label(bars, fmt="%.2f", padding=3)
    axis.axhline(80, color="#444444", linestyle="--", label="100 ms 周期的 P99.9 门限")
    axis.set_xticks(range(5), ["S0 / NTFS", "S4 / NTFS", "S5 / ext4", "S5 / NTFS", "S5 / Windows"])
    axis.set_ylabel("完整周期 P99.9 (ms)")
    axis.set_ylim(0, 130)
    axis.set_title("当前源码的单次开发测量")
    axis.legend(loc="upper left")
    axis.grid(axis="y", alpha=.2)
    save(figure, "cycle-summary")

    figure, axes = plt.subplots(1, 2, figsize=(10, 5), layout="constrained")
    names = ["s4-A", "s4-C"]
    counts = [float(rows[name]["restarts"]) for name in names]
    bars = axes[0].bar(["A", "C"], counts, color=["#286a8e", "#3b9c87"])
    axes[0].bar_label(bars, fmt="%.0f", padding=3)
    axes[0].set_ylabel("恢复事件数量")
    axes[0].set_ylim(0, 12)
    fractions = [float(rows[name]["coverage_fraction"]) * 100 for name in names]
    bars = axes[1].bar(["A", "C"], fractions, color=["#286a8e", "#3b9c87"])
    axes[1].bar_label(bars, fmt="%.2f%%", padding=3)
    axes[1].set_ylim(0, 100)
    axes[1].set_ylabel("路径进度覆盖 (%)")
    figure.suptitle("相同配置的单次 A/C 配对：两组均暂停超时")
    for axis in axes:
        axis.grid(axis="y", alpha=.2)
    save(figure, "recovery-pair")
    (OUTPUT / "figures.json").write_text(json.dumps({"fonts": fonts, "source_sha256": checksum(DATA / "metrics.csv"), "files": exported}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("exported", len(exported), "summary files")


if __name__ == "__main__":
    main()
