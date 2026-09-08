"""Pytest bootstrap for the portable_oscbf package.

Ensures that ``work`` (the portable package itself) and the vendored local
``dpax`` copy are importable before any test module is collected, regardless
of the directory pytest is invoked from.
"""

from __future__ import annotations

import sys
from pathlib import Path


_ROOT = Path(__file__).resolve().parent

# ``vendor/dpax`` is the project root of the vendored dpax distribution
# (``setup.py`` + ``dpax/`` package).  Adding it to sys.path makes
# ``import dpax`` resolve to ``vendor/dpax/dpax`` instead of being shadowed
# by an editable install or accidentally treated as a namespace package.
for _entry in (_ROOT, _ROOT / "vendor" / "dpax"):
    _text = str(_entry)
    if _text not in sys.path:
        sys.path.insert(0, _text)


def _dpax_is_vendored() -> bool:
    """Return True when the top-level ``dpax`` module resolves under vendor/."""

    vendor_root = (_ROOT / "vendor").resolve()
    try:
        import dpax
    except ImportError:
        return False
    return Path(dpax.__file__).resolve().is_relative_to(vendor_root)


def pytest_configure(config):
    """Fail loudly if the vendored dpax is shadowed by an external install."""

    if not _dpax_is_vendored():
        raise RuntimeError(
            "portable_oscbf requires its vendored dpax copy; "
            "an external dpax is being imported instead"
        )
