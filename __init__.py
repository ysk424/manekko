"""Manekko — VIVE Tracker full-body mocap → CC character via mink IK.

Blender 5.1 extension. Bundled wheels (mink/mujoco/openvr/...) are normally put
on sys.path by Blender's extension wheel system; numpy comes from Blender itself.

In practice that auto-install proved unreliable here (no ``extensions/.local``
site-packages was ever created, so ``import openvr`` failed at runtime). So we
self-bootstrap: ``_ensure_wheels()`` extracts the bundled ./wheels/*.whl once
into ./_libs and puts it on sys.path — but only if the deps don't already import
(if Blender did install them, or a dev path is active, it's a no-op). numpy is
never bundled, so it keeps resolving to Blender's own.

N-panel "Manekko" (3D View sidebar) has three buttons: Start/Stop, Record,
Calibrate. See src/ops.py.
"""
from __future__ import annotations

import glob
import os
import sys
import zipfile

import bpy
from bpy.props import BoolProperty
from bpy.types import WindowManager


def _ensure_wheels() -> None:
    """Guarantee the bundled deps are importable, independent of Blender's wheel
    system. No-op if they already import. Extracts ./wheels/*.whl into ./_libs
    once (excluding numpy, which is never bundled) and prepends it to sys.path."""
    try:
        import openvr  # noqa: F401
        import mink     # noqa: F401  (pulls in mujoco)
        return
    except Exception:
        pass

    here = os.path.dirname(__file__)
    wheels_dir = os.path.join(here, "wheels")
    if not os.path.isdir(wheels_dir):
        return  # nothing bundled (shouldn't happen in a built package)

    libs = os.path.join(here, "_libs")
    marker = os.path.join(libs, ".extracted")
    if not os.path.exists(marker):
        os.makedirs(libs, exist_ok=True)
        for whl in sorted(glob.glob(os.path.join(wheels_dir, "*.whl"))):
            try:
                with zipfile.ZipFile(whl) as z:
                    z.extractall(libs)
            except Exception as e:  # noqa: BLE001
                print(f"[manekko] failed to extract {os.path.basename(whl)}: {e!r}")
        with open(marker, "w") as f:
            f.write("ok\n")

    if libs not in sys.path:
        sys.path.insert(0, libs)


_ensure_wheels()

from . import ui            # noqa: E402
from .src import ops        # noqa: E402


def register() -> None:
    WindowManager.manekko_running = BoolProperty(
        name="Manekko Running", default=False, options={"SKIP_SAVE"})
    WindowManager.manekko_recording = BoolProperty(
        name="Manekko Recording", default=False, options={"SKIP_SAVE"})
    ops.register()
    ui.register()


def unregister() -> None:
    ui.unregister()
    ops.unregister()
    for attr in ("manekko_running", "manekko_recording"):
        try:
            delattr(WindowManager, attr)
        except Exception:
            pass
