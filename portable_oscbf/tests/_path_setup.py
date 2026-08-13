"""_path_setup.py — Add project paths to sys.path for direct test execution.

Import this at the top of any test file that needs direct
``python3 tests/test_*.py`` execution::

    import _path_setup  # noqa: F401

This is a no-op when paths are already set (e.g. via pytest + conftest.py).
"""
import sys
import os

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_WORK = os.path.join(_ROOT, "work")

for _p in (_WORK, _ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)
