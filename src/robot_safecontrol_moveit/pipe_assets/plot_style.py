from pathlib import Path

import matplotlib
from matplotlib import font_manager
from matplotlib.ft2font import FT2Font


def configure_fonts():
    records, coverage = {}, {}
    for name in ("Times New Roman", "SimSun"):
        path = Path(font_manager.findfont(
            font_manager.FontProperties(family=name), fallback_to_default=False
        ))
        face = FT2Font(str(path))
        if face.family_name != name:
            raise ValueError(f"字体身份不一致：{name} / {face.family_name}")
        records[name] = {"path": str(path), "family": face.family_name}
        coverage[name] = set(face.get_charmap())
    matplotlib.rcParams.update({
        "font.family": ["Times New Roman", "SimSun"],
        "font.size": 10,
        "axes.unicode_minus": True,
    })
    return records, coverage
