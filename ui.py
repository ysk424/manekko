"""3D View N-panel — Manekko. Three buttons: Start/Stop, Record, Calibrate."""
from __future__ import annotations

import bpy
from bpy.types import Panel


class MANEKKO_PT_main(Panel):
    bl_idname = "MANEKKO_PT_main"
    bl_label = "Manekko"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Manekko"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        wm = context.window_manager
        running = getattr(wm, "manekko_running", False)
        recording = getattr(wm, "manekko_recording", False)

        # 1) Start / Stop (toggle)
        if not running:
            layout.operator("manekko.start", text="Start", icon="PLAY")
        else:
            layout.operator("manekko.stop", text="Stop", icon="SNAP_FACE")

        # 2) Record (toggle)  3) Calibrate — only while running
        col = layout.column(align=True)
        col.enabled = running
        col.operator("manekko.record",
                     text=("Stop Recording" if recording else "Record (5s)"),
                     icon="REC", depress=recording)
        col.operator("manekko.calibrate", text="Calibrate (5s)",
                     icon="FILE_REFRESH")

        if running and not recording:
            layout.label(text="tracking", icon="RADIOBUT_ON")
        elif recording:
            layout.label(text="recording", icon="REC")


_classes = (MANEKKO_PT_main,)


def register() -> None:
    for cls in _classes:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)
