"""Blender modal operator: live-drive the CC character from VIVE trackers.

Validated live 2026-05-30 (full body, all 10 roles, via LiveDriver). Real-time
viewport comes from a `wm` timer that returns control to the event loop each
tick — a blocking Python loop does NOT present frames (see
docs/live_driving_notes.md).

Beep-driven flow (the controller trigger is not usable via the legacy OpenVR
API; see openvr_reader): Start -> beep1 -> 5 s (performer takes the A-pose) ->
beep2 (calibrate) -> drive. Stop with ESC.

Only roles whose tracker was valid at the calibration instant are driven; a
role going invalid later (foot under a desk, controller put down) simply holds
its last target instead of being yanked.

NOTE: not yet wired into the add-on register() / N-panel, and not load-tested as
a packaged extension — only the operator logic is validated. Wiring + a Stop
button are TODO.
"""
from __future__ import annotations

import time

import bpy


def _beep(freq: int, ms: int) -> None:
    try:
        import winsound
        winsound.Beep(freq, ms)
    except Exception:
        pass


def _modules():
    """Return (live, openvr_reader) for both packaged and dev-loop loading."""
    try:
        from . import live as _live, openvr_reader as _ovr  # packaged extension
        return _live, _ovr
    except ImportError:  # dev loop: modules loaded standalone via importlib
        import sys
        return sys.modules["manekko_live"], sys.modules["manekko_openvr"]


def _find_armature(context):
    obj = context.object
    if obj and obj.type == "ARMATURE" and "CC_Base_Hip" in obj.data.bones:
        return obj
    for o in bpy.data.objects:
        if o.type == "ARMATURE" and "CC_Base_Hip" in o.data.bones:
            return o
    return None


def _tag_view3d(context) -> None:
    for w in context.window_manager.windows:
        for a in w.screen.areas:
            if a.type == "VIEW_3D":
                a.tag_redraw()


class MANEKKO_OT_live(bpy.types.Operator):
    bl_idname = "manekko.live"
    bl_label = "Manekko Live"
    bl_description = "Live-drive the character from VIVE trackers (ESC to stop)"

    apose_seconds: bpy.props.FloatProperty(default=5.0)  # type: ignore

    def invoke(self, context, event):
        import openvr
        live, ovr = _modules()
        self._live, self._ovr, self._openvr = live, ovr, openvr

        arm = _find_armature(context)
        if arm is None:
            self.report({"ERROR"}, "No CC_Base armature found")
            return {"CANCELLED"}

        self.vr = openvr.init(openvr.VRApplication_Background)
        self.role_idx = {}
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            cls = self.vr.getTrackedDeviceClass(i)
            if cls in (openvr.TrackedDeviceClass_GenericTracker,
                       openvr.TrackedDeviceClass_Controller):
                try:
                    ser = self.vr.getStringTrackedDeviceProperty(
                        i, openvr.Prop_SerialNumber_String)
                except Exception:
                    ser = None
                role = ovr.SERIAL_TO_ROLE.get(ser)
                if role:
                    self.role_idx[role] = i

        self.driver = live.LiveDriver(arm)
        self.driver.apply_rest()
        context.view_layer.update()

        self.state = "WAIT"
        self.t0 = time.perf_counter()
        _beep(660, 250)  # get into the A-pose
        self.timer = context.window_manager.event_timer_add(
            1.0 / 30.0, window=context.window)
        context.window_manager.modal_handler_add(self)
        self.report({"INFO"}, f"manekko live: A-pose in {self.apose_seconds:.0f}s")
        return {"RUNNING_MODAL"}

    def _snapshot(self, calibrated_only=False):
        openvr, ovr = self._openvr, self._ovr
        poses = self.vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
        snap = {}
        for role, idx in self.role_idx.items():
            p = poses[idx]
            if p.bPoseIsValid:
                m = p.mDeviceToAbsoluteTracking
                snap[role] = ovr.world_to_blender((m[0][3], m[1][3], m[2][3]))
        if calibrated_only and self.driver.calibration is not None:
            snap = {r: v for r, v in snap.items()
                    if r in self.driver.calibration.offset}
        return snap

    def modal(self, context, event):
        if event.type == "ESC" and event.value == "PRESS":
            return self._finish(context)
        if event.type == "TIMER":
            if self.state == "WAIT":
                if time.perf_counter() - self.t0 >= self.apose_seconds:
                    snap = self._snapshot()
                    if snap:
                        self.driver.calibrate(snap)
                        self.state = "DRIVE"
                        _beep(1175, 400)  # calibrated; driving
                        self.report({"INFO"}, "manekko live: DRIVING (ESC to stop)")
            elif self.state == "DRIVE":
                snap = self._snapshot(calibrated_only=True)
                if snap:
                    self.driver.step(snap, iters=4)
                    _tag_view3d(context)
        return {"PASS_THROUGH"}

    def _finish(self, context):
        try:
            context.window_manager.event_timer_remove(self.timer)
        except Exception:
            pass
        try:
            self.driver.apply_rest()
            context.view_layer.update()
        except Exception:
            pass
        try:
            self._openvr.shutdown()
        except Exception:
            pass
        _beep(988, 200)
        self.report({"INFO"}, "manekko live stopped")
        return {"CANCELLED"}


def register():
    bpy.utils.register_class(MANEKKO_OT_live)


def unregister():
    bpy.utils.unregister_class(MANEKKO_OT_live)
