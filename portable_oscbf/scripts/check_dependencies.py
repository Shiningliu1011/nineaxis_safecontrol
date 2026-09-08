#!/usr/bin/env python3
"""M0 dependency self-check for portable_oscbf.

Verifies that every required third-party package is importable and that the
top-level ``dpax`` module resolves to the local vendored copy.  Exit code 0
means all checks passed.

Usage::

    python3 portable_oscbf/scripts/check_dependencies.py
"""

from __future__ import annotations

import importlib
import importlib.metadata
import sys
from pathlib import Path


_ROOT = Path(__file__).resolve().parents[1]

# ``work`` must be importable and the vendored dpax must win over any
# site-packages editable install before any dependency is imported.
# ``vendor/dpax`` is the dpax distribution root (setup.py + dpax/ package).
for _entry in (_ROOT, _ROOT / "vendor" / "dpax"):
    _text = str(_entry)
    if _text not in sys.path:
        sys.path.insert(0, _text)


REQUIRED = (
    "numpy",
    "scipy",
    "yaml",
    "jax",
    "jaxlib",
    "cbfpy",
    "qpax",
    "fcl",
    "trimesh",
    "dpax",
)


def _version(name: str, module) -> str:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return getattr(module, "__version__", "?")


def main() -> int:
    ok = True
    for name in REQUIRED:
        try:
            module = importlib.import_module(name)
        except Exception as exc:  # pragma: no cover - failure path
            ok = False
            print(f"FAIL  {name:10s} import error: {exc}")
            continue
        print(f"OK    {name:10s} version={_version(name, module)}  {module.__file__}")

    import dpax

    dpax_file = Path(dpax.__file__).resolve()
    vendor_root = (_ROOT / "vendor").resolve()
    if dpax_file.is_relative_to(vendor_root):
        print(f"OK    dpax         vendored -> {dpax_file}")
    else:
        ok = False
        print(f"FAIL  dpax         resolves outside vendor/ -> {dpax_file}")

    try:
        import work

        print(f"OK    work         importable -> {work.__file__}")
    except Exception as exc:  # pragma: no cover - failure path
        ok = False
        print(f"FAIL  work         import error: {exc}")

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
