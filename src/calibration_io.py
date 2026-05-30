"""Persist the A-pose calibration to a JSON file in the user config dir.

Saved to Blender's user CONFIG resource (``.../config/manekko/calibration.json``)
so it survives extension reinstalls / Blender updates and is always writable.
Auto-loaded on Start so the performer needn't recalibrate every session (a
fresh Calibrate overwrites it). Stores the per-role position offsets; the
detailed/robust calibration is a later refinement.
"""
from __future__ import annotations

import json
import os

import numpy as np
import bpy

from . import openvr_reader as _ovr


def _path(create: bool = False) -> str:
    d = bpy.utils.user_resource("CONFIG", path="manekko", create=create)
    return os.path.join(d, "calibration.json")


def save(arm, calibration) -> str | None:
    if calibration is None:
        return None
    data = {
        "armature": getattr(arm, "name", ""),
        "offset": {role: [float(x) for x in vec]
                   for role, vec in calibration.offset.items()},
    }
    path = _path(create=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    return path


def load(arm) -> "_ovr.Calibration | None":
    path = _path()
    if not os.path.exists(path):
        return None
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        offset = {role: np.asarray(vec, dtype=float)
                  for role, vec in data.get("offset", {}).items()}
        if not offset:
            return None
        return _ovr.Calibration(offset=offset)
    except Exception:
        return None
