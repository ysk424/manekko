"""3D View N-panel — Manekko."""
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
        mode = getattr(wm, "manekko_mode", "IDLE")
        layout.label(text=f"Mode: {mode}")
        layout.label(text="(scaffold — IK core pending)")


_classes = (MANEKKO_PT_main,)


def register() -> None:
    for cls in _classes:
        bpy.utils.register_class(cls)


def unregister() -> None:
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)
