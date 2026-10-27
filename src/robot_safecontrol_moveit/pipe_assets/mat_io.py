from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np
from pymatreader import read_mat
from scipy.io import loadmat, savemat
from scipy.io.matlab import matfile_version


def load_mat(path: str | Path) -> dict[str, Any]:
    filename = Path(path)
    if not filename.is_file():
        raise FileNotFoundError(filename)
    if filename.suffix.lower() != ".mat":
        raise ValueError(f"需要 MAT 文件：{filename}")
    if matfile_version(filename)[0] == 2:
        data = read_mat(filename)
    else:
        data = loadmat(filename, simplify_cells=True)
    return {name: value for name, value in data.items() if not name.startswith("__")}


def save_mat(path: str | Path, data: Mapping[str, Any]) -> Path:
    filename = Path(path)
    if filename.suffix.lower() != ".mat":
        raise ValueError(f"需要 MAT 文件：{filename}")
    if not filename.parent.is_dir():
        raise FileNotFoundError(filename.parent)
    if not data or any(not name or name.startswith("_") for name in data):
        raise ValueError("MAT 变量名称必须非空，且不能以下划线开头")
    savemat(filename, _mat_value(dict(data)), long_field_names=True, oned_as="column")
    return filename


def _mat_value(value):
    if value is None:
        return np.empty(0)
    if isinstance(value, Mapping):
        return {name: _mat_value(item) for name, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_mat_value(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    return value
