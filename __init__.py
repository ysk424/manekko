"""Manekko — VIVE Tracker full-body mocap → CC character via mink IK.

Blender 5.1 extension. Bundled wheels (mink/mujoco/openvr/...) are placed on
sys.path by Blender's extension wheel system; numpy comes from Blender itself.

N-panel "Manekko" (3D View sidebar) has three buttons: Start/Stop, Record,
Calibrate. See src/ops.py.
"""
from __future__ import annotations

import bpy
from bpy.props import BoolProperty
from bpy.types import WindowManager

from . import ui
from .src import ops


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
