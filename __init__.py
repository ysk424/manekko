"""Manekko — VIVE Tracker full-body mocap → CC character via mink IK.

Blender 5.1 extension. Bundled wheels (mink/mujoco/openvr/...) are placed on
sys.path by Blender's extension wheel system; numpy comes from Blender itself.
"""
from __future__ import annotations

import bpy
from bpy.props import EnumProperty
from bpy.types import WindowManager

from . import ui


MODE_IDLE = "IDLE"        # extension inactive
MODE_LIVE = "LIVE"        # live-driving the character from trackers
MODE_RECORD = "RECORD"    # live-driving AND baking keyframes to an Action


_classes: tuple = ()


def register() -> None:
    for cls in _classes:
        bpy.utils.register_class(cls)
    WindowManager.manekko_mode = EnumProperty(
        name="Manekko Mode",
        items=[
            (MODE_IDLE, "Idle", "Extension inactive"),
            (MODE_LIVE, "Live", "Drive the character live from trackers"),
            (MODE_RECORD, "Record", "Drive live and bake keyframes to an Action"),
        ],
        default=MODE_IDLE,
        options={"SKIP_SAVE"},
    )
    ui.register()


def unregister() -> None:
    ui.unregister()
    try:
        del WindowManager.manekko_mode
    except Exception:
        pass
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)
