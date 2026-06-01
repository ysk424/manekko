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

# Option-B spike (v0.1.1): legacy controller buttons/axes (trigger, trackpad)
# were dead under VRApplication_Background. Try Overlay (coexists with the scene
# app, still receives poses) to see if legacy getControllerState wakes up; fall
# back to Background if Overlay init fails. The pose/IK path is unchanged — this
# only adds an observable controller readout in the N-panel. Set to "background"
# to revert the app-type change.
OPENVR_APP_TYPE = "overlay"   # "overlay" | "background"

# Shared state between the running modal and the button operators (one session).
#   mode      : "calib" | "record"  (left MENU toggles)
#   rec_state : "idle" | "playing" | "recording" (right MENU cycles, record mode)
#   btn_prev  : per-controller MENU pressed state, for rising-edge detection
#   punch_*   : recorded segment boundary frames, kept for batch smoothing
#   audio_done: WAV cue already fired this playback
_S = {"stop": False, "calib_at": None, "record_at": None, "recording": False,
      "ctrl": {}, "app_type": "",
      "mode": "calib", "rec_state": "idle",
      "btn_prev": {"palm_l": False, "palm_r": False},
      "punch_in": None, "punch_out": None, "audio_done": False}

# Controller button mask (VIVE wand): ApplicationMenu = bit 1 (see
# docs/hands_and_input.md). Left controller = palm_l, right = palm_r.
MENU_MASK = 0x2


def _init_vr(openvr):
    """Init OpenVR, trying OPENVR_APP_TYPE first (Overlay for the legacy-input
    spike) then falling back to Background. Returns (vr, type_name)."""
    types = {"overlay": openvr.VRApplication_Overlay,
             "background": openvr.VRApplication_Background}
    order = ["overlay", "background"] if OPENVR_APP_TYPE == "overlay" else ["background"]
    last = None
    for name in order:
        try:
            return openvr.init(types[name]), name
        except Exception as e:  # noqa: BLE001
            last = e
    raise last


def _beep(freq: int, ms: int) -> None:
    try:
        import winsound
        winsound.Beep(freq, ms)
    except Exception:
        pass


def _mods():
    from . import live as _live, openvr_reader as _ovr, calibration_io as _cio
    return _live, _ovr, _cio


def _load_wav(path):
    """Preload a WAV as an aud.Sound (no-op-safe). Returns (device, sound) or
    (None, None). Done up front so playback at the cue frame has no latency."""
    if not path:
        return None, None
    try:
        import os
        import aud
        p = bpy.path.abspath(path)
        if not os.path.isfile(p):
            return None, None
        return aud.Device(), aud.Sound(p)
    except Exception:  # noqa: BLE001
        return None, None


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
            self.vr, app_type = _init_vr(openvr)
        except Exception as e:
            self.report({"ERROR"}, f"SteamVR not available: {e}")
            return {"CANCELLED"}
        _S["app_type"] = app_type

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
                   "recording": False, "mode": "calib", "rec_state": "idle",
                   "btn_prev": {"palm_l": False, "palm_r": False},
                   "punch_in": None, "punch_out": None, "audio_done": False})
        self._old_seg = {}
        wm = context.window_manager
        wm.manekko_running = True
        wm.manekko_recording = False
        wm.manekko_mode = "CALIB"
        # Preload the cue WAV (frame fps*10) so playback has no startup latency.
        self._aud_dev, self._aud_snd = _load_wav(getattr(wm, "manekko_wav_path", ""))
        self._aud_handle = None
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
        pos_snap, rot_snap = {}, {}
        for role, idx in self.role_idx.items():
            p = poses[idx]
            if p.bPoseIsValid:
                m = p.mDeviceToAbsoluteTracking
                R = ovr._mat34_rot(m)
                # Push the raw tracker position along its local normal toward the
                # bone (BONE_OFFSET_M[role]) in the SteamVR frame, BEFORE the axis
                # swap — same as openvr_reader._loop. This was previously skipped
                # here, so the bone offsets were dead in the live extension (the
                # offsets only ran in the unused TrackerReader thread).
                pos = ovr.correct_to_bone(role, R, ovr._mat34_pos(m))
                pos_snap[role] = ovr.world_to_blender(pos)
                # Orientation in the Blender world frame (same axis map); used
                # only by orientation-tracked roles (head, hip) after A-pose
                # registration in Calibration.rot_offset.
                rot_snap[role] = ovr.world_rot_to_blender(R)
        return pos_snap, rot_snap

    def _drive_snapshot(self, valid):
        cal = self.driver.calibration
        if cal is None:
            return {}
        return {r: v for r, v in valid.items() if r in cal.offset}

    def _read_buttons(self):
        """Option-B spike: read legacy controller state for the palm controllers
        so the N-panel can show whether trigger/trackpad/buttons are live (packet
        number advancing + axis values changing = legacy input works)."""
        out = {}
        for role in ("palm_l", "palm_r"):
            idx = self.role_idx.get(role)
            if idx is None:
                continue
            try:
                ok, st = self.vr.getControllerState(idx)
            except Exception as e:  # noqa: BLE001
                out[role] = {"err": repr(e)}
                continue
            if not ok:
                out[role] = {"invalid": True}
                continue
            out[role] = {
                "pkt": int(st.unPacketNum),
                "btn": int(st.ulButtonPressed),
                # VIVE wand: axis0 = trackpad (x,y), axis1 = trigger (x in 0..1).
                "axes": [(round(st.rAxis[i].x, 3), round(st.rAxis[i].y, 3))
                         for i in range(5)],
            }
        return out

    def _grip_from_ctrl(self, ctrl):
        """Controller trigger (axis1.x, 0..1) -> per-hand finger curl. Maps the
        palm controller to the same-side hand."""
        out = {}
        for palm, hand in (("palm_l", "hand_l"), ("palm_r", "hand_r")):
            d = ctrl.get(palm)
            if d and "axes" in d:
                out[hand] = max(0.0, min(1.0, float(d["axes"][1][0])))
        return out

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
        # finger bones (trigger-driven curl, rotation_euler) — capture the grip too
        for names in self.driver.finger_names.values():
            for bn in names:
                pb = arm.pose.bones.get(bn)
                if pb is not None:
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
            valid, valid_rot = self._read_valid()
            _S["ctrl"] = self._read_buttons()   # option-B spike readout
            self._handle_menu_buttons(context, valid, valid_rot)

            if _S["calib_at"] is not None and now >= _S["calib_at"]:
                _S["calib_at"] = None
                if valid:
                    self.driver.calibrate(valid, valid_rot)
                    self._cio.save(self.driver.arm, self.driver.calibration)
                    _beep(1175, 400)

            if _S["record_at"] is not None and now >= _S["record_at"]:
                _S["record_at"] = None
                _S["recording"] = True
                wm.manekko_recording = True
                _beep(1175, 400)

            self._maybe_play_audio(context)

            if _S["mode"] == "record" and _S["rec_state"] == "playing":
                # Pre-punch-in: play back the existing take (no live drive, no
                # keys) so the performer can time the punch-in to the cue.
                f = context.scene.frame_current
                if f < context.scene.frame_end:
                    context.scene.frame_set(f + 1)
                _redraw(context)
            else:
                snap = self._drive_snapshot(valid)
                if snap:
                    grip = self._grip_from_ctrl(_S["ctrl"])
                    self.driver.step(snap, valid_rot, grip, iters=4)
                    if _S["recording"]:
                        self._keyframe(context)
                        context.scene.frame_set(context.scene.frame_current + 1)
                    _redraw(context)
        return {"PASS_THROUGH"}

    # -- controller MENU workflow (v0.2.0) ------------------------------
    def _handle_menu_buttons(self, context, valid, valid_rot):
        """Rising-edge on each controller's ApplicationMenu button. Left =
        mode toggle; right = act within the current mode."""
        ctrl = _S["ctrl"]
        for role, side in (("palm_l", "left"), ("palm_r", "right")):
            d = ctrl.get(role)
            pressed = bool(d and "btn" in d and (int(d["btn"]) & MENU_MASK))
            prev = _S["btn_prev"].get(role, False)
            _S["btn_prev"][role] = pressed
            if pressed and not prev:
                if side == "left":
                    self._on_left_menu(context)
                else:
                    self._on_right_menu(context, valid, valid_rot)

    def _on_left_menu(self, context):
        if _S["mode"] == "calib":
            _S["mode"] = "record"
            self._enter_record_mode(context)
        else:
            # leaving record mode: stop & smooth any in-progress take
            if _S["rec_state"] == "recording":
                self._end_recording(context)
            self._stop_audio()
            _S["mode"] = "calib"
            _S["rec_state"] = "idle"
        context.window_manager.manekko_mode = _S["mode"].upper()
        _beep(784, 120)

    def _enter_record_mode(self, context):
        """Rewind to frame 1 and re-arm the right-button cycle (this is also the
        retake reset: toggle calib->record to rewind & re-arm)."""
        self._stop_audio()
        context.scene.frame_set(1)
        _S.update({"rec_state": "idle", "recording": False,
                   "punch_in": None, "punch_out": None, "audio_done": False})
        self._old_seg = {}
        wm = context.window_manager
        wm.manekko_recording = False
        # pick up a WAV path changed since Start (preload again, no latency)
        self._aud_dev, self._aud_snd = _load_wav(getattr(wm, "manekko_wav_path", ""))

    def _on_right_menu(self, context, valid, valid_rot):
        if _S["mode"] == "calib":
            if valid:
                self.driver.calibrate(valid, valid_rot)
                self._cio.save(self.driver.arm, self.driver.calibration)
                _beep(1175, 400)
            return
        st = _S["rec_state"]
        if st == "idle":                       # press 1: start playback @ 1F
            self._stop_audio()
            context.scene.frame_set(1)
            _S["audio_done"] = False
            _S["rec_state"] = "playing"
            _beep(660, 200)
        elif st == "playing":                  # press 2: punch-in
            self._begin_recording(context)
        elif st == "recording":                # press 3: punch-out
            self._end_recording(context)

    def _action_fcurves(self):
        """(action, [fcurves]) for the armature's active action. Handles both
        legacy actions (``action.fcurves``) and Blender 4.4+ slotted actions
        (fcurves live in the active slot's channelbag, ``action.fcurves`` gone)."""
        arm = self.driver.arm
        adt = getattr(arm, "animation_data", None)
        if not adt or not adt.action:
            return None, []
        act = adt.action
        fcurves = []
        slot = getattr(adt, "action_slot", None)
        try:
            for layer in act.layers:
                for strip in layer.strips:
                    try:
                        bag = strip.channelbag(slot) if slot is not None else None
                    except Exception:  # noqa: BLE001
                        bag = None
                    if bag is not None:
                        fcurves.extend(bag.fcurves)
        except Exception:  # noqa: BLE001
            pass
        if not fcurves and hasattr(act, "fcurves"):
            try:
                fcurves = list(act.fcurves)
            except Exception:  # noqa: BLE001
                fcurves = []
        return act, fcurves

    def _begin_recording(self, context):
        f = context.scene.frame_current
        _S["punch_in"] = f
        # preserve the existing take over the to-be-overwritten region so the
        # punch-in/out crossfade has the 'old' curve to blend against
        from . import postproc
        _, fcurves = self._action_fcurves()
        self._old_seg = postproc.sample_segment(fcurves, f)
        _S["recording"] = True
        _S["rec_state"] = "recording"
        context.window_manager.manekko_recording = True
        _beep(1175, 400)

    def _end_recording(self, context):
        _S["punch_out"] = context.scene.frame_current
        _S["recording"] = False
        _S["rec_state"] = "idle"
        context.window_manager.manekko_recording = False
        self._stop_audio()
        _beep(988, 200)
        try:
            from . import postproc
            fps = max(1, int(getattr(context.scene.render, "fps", 24)))
            _, fcurves = self._action_fcurves()
            postproc.smooth_take(
                self.driver.arm, self.driver,
                fcurves=fcurves, fps=fps,
                punch_in=_S["punch_in"], punch_out=_S["punch_out"],
                old_seg=getattr(self, "_old_seg", {}),
                smooth_frames=getattr(context.window_manager,
                                      "manekko_smooth_frames", 6))
            context.view_layer.update()
        except Exception as e:  # noqa: BLE001
            self.report({"WARNING"}, f"smooth failed: {e!r}")

    def _maybe_play_audio(self, context):
        """Fire the cue WAV once, when the timeline crosses frame fps*10 during
        record-mode playback or recording."""
        if _S.get("audio_done") or _S["mode"] != "record":
            return
        if _S["rec_state"] not in ("playing", "recording"):
            return
        fps = max(1, int(getattr(context.scene.render, "fps", 24)))
        if context.scene.frame_current >= fps * 10 and self._aud_snd is not None:
            try:
                self._aud_handle = self._aud_dev.play(self._aud_snd)
            except Exception:  # noqa: BLE001
                pass
            _S["audio_done"] = True

    def _stop_audio(self):
        """Halt the cue WAV (on Stop / punch-out / rewind), so it doesn't keep
        playing past the take."""
        h = getattr(self, "_aud_handle", None)
        if h is not None:
            try:
                h.stop()
            except Exception:  # noqa: BLE001
                pass
        self._aud_handle = None
        _S["audio_done"] = False

    def _finish(self, context):
        self._stop_audio()
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
                   "recording": False, "mode": "calib", "rec_state": "idle",
                   "punch_in": None, "punch_out": None, "audio_done": False})
        wm = context.window_manager
        wm.manekko_running = False
        wm.manekko_recording = False
        wm.manekko_mode = "CALIB"
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
