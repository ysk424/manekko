"""Manekko operators: Start/Stop tracking, Record (toggle), Calibrate.

One modal operator (Start) owns the OpenVR session + LiveDriver and runs the
live loop on a ``wm`` timer (real-time viewport; a blocking loop never presents
frames). The N-panel buttons are thin operators that flip shared state which the
modal reads each tick:

* Start/Stop  : Start launches the modal (immediate). Stop ends it.
* Record      : toggle. Press -> beep -> 5 s -> beep, recording begins
                (keyframes the driven bones onto the active Action, advancing the
                timeline one frame per captured tick). Press again -> stop.
* Calibrate   : press -> beep -> 5 s -> beep, captures the A-pose and saves the
                calibration JSON (auto-loaded next Start).

Record/Calibrate are only meaningful while tracking is running (Start).
"""
from __future__ import annotations

import time

import bpy

COUNTDOWN = 5.0

# Shared state between the running modal and the button operators (one session).
_S = {"stop": False, "calib_at": None, "record_at": None, "recording": False}


def _beep(freq: int, ms: int) -> None:
    try:
        import winsound
        winsound.Beep(freq, ms)
    except Exception:
        pass


def _mods():
    from . import live as _live, openvr_reader as _ovr, calibration_io as _cio
    return _live, _ovr, _cio


def _find_armature(context):
    obj = context.object
    if obj and obj.type == "ARMATURE" and "CC_Base_Hip" in obj.data.bones:
        return obj
    for o in bpy.data.objects:
        if o.type == "ARMATURE" and "CC_Base_Hip" in o.data.bones:
            return o
    return None


class MANEKKO_OT_start(bpy.types.Operator):
    bl_idname = "manekko.start"
    bl_label = "Start"
    bl_description = "Start live-driving the character from VIVE trackers"

    def invoke(self, context, event):
        import openvr
        live, ovr, cio = _mods()
        arm = _find_armature(context)
        if arm is None:
            self.report({"ERROR"}, "No CC_Base armature found")
            return {"CANCELLED"}
        try:
            self.vr = openvr.init(openvr.VRApplication_Background)
        except Exception as e:
            self.report({"ERROR"}, f"SteamVR not available: {e}")
            return {"CANCELLED"}

        self._openvr, self._ovr, self._cio = openvr, ovr, cio
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
        cal = cio.load(arm)            # auto-load saved calibration if present
        if cal is not None:
            self.driver.calibration = cal
        self.driver.apply_rest()
        context.view_layer.update()

        _S.update({"stop": False, "calib_at": None, "record_at": None,
                   "recording": False})
        wm = context.window_manager
        wm.manekko_running = True
        wm.manekko_recording = False
        fps = max(1, int(getattr(context.scene.render, "fps", 30)))
        self.timer = wm.event_timer_add(1.0 / fps, window=context.window)
        wm.modal_handler_add(self)
        msg = "running" if cal is not None else "running (no calibration — Calibrate first)"
        self.report({"INFO"}, f"manekko {msg}")
        return {"RUNNING_MODAL"}

    # -- snapshots ------------------------------------------------------
    def _read_valid(self):
        ovr, openvr = self._ovr, self._openvr
        poses = self.vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
        snap = {}
        for role, idx in self.role_idx.items():
            p = poses[idx]
            if p.bPoseIsValid:
                m = p.mDeviceToAbsoluteTracking
                # Push the raw tracker position along its local normal toward the
                # bone (BONE_OFFSET_M[role]) in the SteamVR frame, BEFORE the axis
                # swap — same as openvr_reader._loop. This was previously skipped
                # here, so the bone offsets were dead in the live extension (the
                # offsets only ran in the unused TrackerReader thread).
                pos = ovr.correct_to_bone(role, ovr._mat34_rot(m), ovr._mat34_pos(m))
                snap[role] = ovr.world_to_blender(pos)
        return snap

    def _drive_snapshot(self, valid):
        cal = self.driver.calibration
        if cal is None:
            return {}
        return {r: v for r, v in valid.items() if r in cal.offset}

    # -- recording ------------------------------------------------------
    def _keyframe(self, context):
        arm, rm = self.driver.arm, self.driver.rm
        frame = context.scene.frame_current
        for body in rm.bodies:
            pb = arm.pose.bones[rm.body_to_bone[body]]
            pb.keyframe_insert("location", frame=frame)
            if pb.rotation_mode == "QUATERNION":
                pb.keyframe_insert("rotation_quaternion", frame=frame)
            else:
                pb.keyframe_insert("rotation_euler", frame=frame)

    # -- modal loop -----------------------------------------------------
    def modal(self, context, event):
        wm = context.window_manager
        if _S["stop"] or not wm.manekko_running:
            return self._finish(context)
        if event.type == "ESC" and event.value == "PRESS":
            return self._finish(context)
        if event.type == "TIMER":
            now = time.perf_counter()
            valid = self._read_valid()

            if _S["calib_at"] is not None and now >= _S["calib_at"]:
                _S["calib_at"] = None
                if valid:
                    self.driver.calibrate(valid)
                    self._cio.save(self.driver.arm, self.driver.calibration)
                    _beep(1175, 400)

            if _S["record_at"] is not None and now >= _S["record_at"]:
                _S["record_at"] = None
                _S["recording"] = True
                wm.manekko_recording = True
                _beep(1175, 400)

            snap = self._drive_snapshot(valid)
            if snap:
                self.driver.step(snap, iters=4)
                if _S["recording"]:
                    self._keyframe(context)
                    context.scene.frame_set(context.scene.frame_current + 1)
                _redraw(context)
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
        _S.update({"stop": False, "calib_at": None, "record_at": None,
                   "recording": False})
        wm = context.window_manager
        wm.manekko_running = False
        wm.manekko_recording = False
        _beep(988, 200)
        return {"CANCELLED"}


class MANEKKO_OT_stop(bpy.types.Operator):
    bl_idname = "manekko.stop"
    bl_label = "Stop"
    bl_description = "Stop live-driving"

    def execute(self, context):
        context.window_manager.manekko_running = False  # modal sees this and ends
        return {"FINISHED"}


class MANEKKO_OT_record(bpy.types.Operator):
    bl_idname = "manekko.record"
    bl_label = "Record"
    bl_description = "Toggle recording (press -> 5 s -> record onto the active Action)"

    def execute(self, context):
        if not context.window_manager.manekko_running:
            self.report({"WARNING"}, "Start first")
            return {"CANCELLED"}
        if _S["recording"]:
            _S["recording"] = False
            context.window_manager.manekko_recording = False
            _beep(988, 200)
        elif _S["record_at"] is None:
            _S["record_at"] = time.perf_counter() + COUNTDOWN
            _beep(660, 250)
        return {"FINISHED"}


class MANEKKO_OT_calibrate(bpy.types.Operator):
    bl_idname = "manekko.calibrate"
    bl_label = "Calibrate"
    bl_description = "Recalibrate the A-pose (press -> 5 s -> capture & save)"

    def execute(self, context):
        if not context.window_manager.manekko_running:
            self.report({"WARNING"}, "Start first")
            return {"CANCELLED"}
        _S["calib_at"] = time.perf_counter() + COUNTDOWN
        _beep(660, 250)
        return {"FINISHED"}


def _redraw(context):
    for w in context.window_manager.windows:
        for a in w.screen.areas:
            if a.type == "VIEW_3D":
                a.tag_redraw()


_classes = (MANEKKO_OT_start, MANEKKO_OT_stop, MANEKKO_OT_record,
            MANEKKO_OT_calibrate)


def register():
    for c in _classes:
        bpy.utils.register_class(c)


def unregister():
    for c in reversed(_classes):
        bpy.utils.unregister_class(c)
