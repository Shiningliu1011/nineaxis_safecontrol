"""Livox point tag filtering shared by real and simulated driver paths."""

from __future__ import annotations

from types import MappingProxyType
from typing import Any, Mapping


# Keep the historical MID-360 behavior: points whose tag bit0..1 is non-zero
# are dropped unless the operator explicitly changes the per-device policy.
DEFAULT_LIVOX_TAG_FILTER: Mapping[str, bool] = MappingProxyType(
    {
        "dropGlue": True,
        "dropRainFogDust": False,
        "dropOther": False,
    }
)

# MID-360 Ethernet protocol, Tag Information:
# bit0..1 = glue point cloud between adjacent objects
# bit2..3 = rain/fog/dust point cloud
# bit4..5 = other detection abnormality
# bit6..7 = reserved (never filtered by these user-facing groups)
_TAG_GROUP_MASKS = {
    "dropGlue": 0x03,
    "dropRainFogDust": 0x0C,
    "dropOther": 0x30,
}


def livox_tag_keep_mask(tags: Any, policy: Mapping[str, bool]) -> Any:
    """Return a boolean mask selecting tags allowed by ``policy``.

    ``tags`` may be any one-dimensional NumPy-compatible sequence. Values are
    interpreted as raw uint8 so reserved bit6..7 survive unchanged.
    """

    import numpy as np

    values = np.asarray(tags, dtype=np.uint8)
    rejected = np.zeros(values.shape, dtype=bool)
    for key, mask in _TAG_GROUP_MASKS.items():
        if policy.get(key, False):
            rejected |= (values & np.uint8(mask)) != 0
    return ~rejected


__all__ = ("DEFAULT_LIVOX_TAG_FILTER", "livox_tag_keep_mask")
