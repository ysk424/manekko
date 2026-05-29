# Live driving — verified findings (2026-05-30 morning)

Hardware-in-the-loop session. Everything below was verified live against real
VIVE Tracker 3.0 + controllers on SteamVR. This is the "outside world" contact
point (vs. the internal IK that was already solid), so it was done carefully and
in small steps. Read this before resuming the live/modal work (task #3).

## 1. Live viewport MUST be a modal operator, NOT a blocking loop

A blocking Python loop (sleep + per-frame `wm.redraw_timer`) does **not** present
frames to the screen — the OS/Blender event loop is starved, so the user sees
nothing until the script returns. The bone data DID move (confirmed by reading
back `(arm.matrix_world @ pose_bone.matrix).translation` == target), and a single
final redraw after the script returns shows the moved state. But continuous live
motion needs a **modal operator + `wm.event_timer_add`** that returns control to
the event loop each tick (this is how Rokoko Studio Live etc. work — confirmed
real-time viewport mocap is possible in Blender, it is not a game-engine-only
thing). Perf tip from Rokoko: "Hide Meshes during Play", close the keyframe
window.

## 2. Coordinate transform — FINAL (front-aligned)

- `svr_to_blender(p) = (x, -z, y)` is the pure Y-up→Z-up axis swap (scale 1,
  determinant +1 → **no mirror**).
- The room has the **screen at world -X** and the performer **faces the screen**,
  so "forward" = world -X must map to the character front (Blender -Y).
  svr_to_blender alone gives world -X → Blender -X (90° off), so apply a fixed
  **+90° yaw about Z** afterwards.
- **Net combined: SteamVR (x, y, z) → Blender (z, x, y)** = `world_to_blender()`
  in `src/openvr_reader.py`. Live-verified: forward→-Y, performer-left→+X. ✓
- Apply consistently to the calibration ref AND live positions (the delta drives
  the character). If the room layout changes, derive the yaw from the hip tracker
  heading instead of hardcoding +90°.

### Rotation-vs-mirror diagnosis (how we know it is a yaw, not a left/right swap)
A 3-point walk test (origin → "-X" → "-Y", captured at beeps) gave signed angle
vecX→vecY = **+87.4°** (ideal +90 → no mirror) and two consistent yaw estimates
(+42°, +44.6°). Single rotation explains everything; no mirror/shear. The earlier
"left/right looks wrong" was purely the yaw.

## 3. Third-person confirmed; the "first-person" scare was a transient

Driving only the **Hip (root) bone** translates the whole character (rigid) —
this is correct third-person behavior. One earlier run *looked* first-person
(view + grid moved with the hips). State inspection found **no** view Lock-to-
Object/Bone, **no** active VR/OpenXR session, no camera parenting — i.e. it was a
transient (likely a since-closed VR preview). The clean re-test logged hip world
pos vs viewport `view_location` and confirmed: character moves, camera static.

## 4. Trigger button — legacy API is dead, IVRInput deferred

`IVRSystem.getControllerState` returns **no live data** (packet number frozen,
all-zero) even when the controller is awake (`activity_level = UserInteraction`).
Modern SteamVR routes input through the action-based **IVRInput** API (action
manifest JSON + bindings + `updateActionState` / `getDigitalActionData`).
Implementing IVRInput is deferred. Until then, **recording is beep-driven**:
`start → beep1 → 5 s (performer takes A-pose) → beep2 (calibrate + start record)`,
stop via the extension button (not the trigger). beeps use `winsound.Beep` on
Windows.

## 5. Device roster (confirmed by floor line-up, see SERIAL_TO_ROLE)

Hands are the **two controllers** (R=hand_r, L=hand_l); the other 8 roles are
trackers (head, hip, elbows, knees, feet). HMD `LHR-CD5AF0FE` is reference only.
Lighthouses enumerate as class 5 (DisplayRedirect) on this rig.

## 6. Validated modal prototype (hip → root, third-person, +90° transform)

This ran live and worked. It is the seed for the real live driver. To extend:
add the other 9 roles as mink IK position targets (full body), set root facing,
add Action keyframing in a RECORD state, and a stop button + N-panel UI.

```python
import bpy, openvr, time, mathutils
import winsound
SER = "LHR-15E5788A"  # hip tracker
def transform(v):                       # SteamVR(x,y,z) -> Blender(z,x,y)
    return mathutils.Vector((v[2], v[0], v[1]))

class MANEKKO_OT_hiptest(bpy.types.Operator):
    bl_idname = "manekko.hiptest"; bl_label = "Manekko Hip Test"
    def invoke(self, context, event):
        self.vr = openvr.init(openvr.VRApplication_Background)
        self.idx = next(i for i in range(openvr.k_unMaxTrackedDeviceCount)
                        if self.vr.getTrackedDeviceClass(i) == openvr.TrackedDeviceClass_GenericTracker
                        and self.vr.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String) == SER)
        self.arm = bpy.data.objects["Toon Neutral_F"]
        self.pb = self.arm.pose.bones["CC_Base_Hip"]
        self.mw = self.arm.matrix_world; self.mw_inv = self.mw.inverted()
        rw = self.mw @ self.arm.data.bones["CC_Base_Hip"].matrix_local
        self.rest_rot = rw.to_quaternion(); self.rest_pos = rw.translation.copy()
        self.state = "WAIT"; self.t0 = time.perf_counter(); self.ref = None
        winsound.Beep(660, 250)
        self.timer = context.window_manager.event_timer_add(1/30, window=context.window)
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}
    def hip(self):
        p = self.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                                    openvr.k_unMaxTrackedDeviceCount)[self.idx]
        m = p.mDeviceToAbsoluteTracking
        return bool(p.bPoseIsValid), transform((m[0][3], m[1][3], m[2][3]))
    def modal(self, context, event):
        if event.type == 'ESC' and event.value == 'PRESS':
            return self.finish(context)
        if event.type == 'TIMER':
            if self.state == "WAIT":
                if time.perf_counter() - self.t0 >= 5.0:
                    v, pos = self.hip()
                    if v:
                        self.ref = pos; self.state = "DRIVE"; winsound.Beep(1175, 400)
            elif self.state == "DRIVE":
                v, pos = self.hip()
                if v:
                    target = self.rest_pos + (pos - self.ref)
                    world = mathutils.Matrix.Translation(target) @ self.rest_rot.to_matrix().to_4x4()
                    loc, rot, _ = (self.mw_inv @ world).decompose()
                    self.pb.matrix = mathutils.Matrix.LocRotScale(loc, rot, mathutils.Vector((1, 1, 1)))
                    context.view_layer.update()
                    for w in context.window_manager.windows:
                        for a in w.screen.areas:
                            if a.type == 'VIEW_3D':
                                a.tag_redraw()
        return {'PASS_THROUGH'}
    def finish(self, context):
        context.window_manager.event_timer_remove(self.timer)
        self.pb.matrix_basis = mathutils.Matrix.Identity(4); context.view_layer.update()
        openvr.shutdown(); winsound.Beep(988, 200)
        return {'CANCELLED'}
```

Notes: root translation uses the same scale-stripped `pb.matrix` path as
`apply.py` (decompose → `LocRotScale(loc, rot, 1)`) so the armature's 0.01 object
scale never leaks onto the root. ESC resets to rest and shuts OpenVR down.

## Next steps (task #3)

1. Promote this modal into the real extension: full-body IK (10 position targets
   via mink), not just root translation — feed `world_to_blender` positions into
   `LiveDriver` (calibrate at beep2), apply joint angles via `apply.py`.
2. Root facing: position is already the only thing on the root; add yaw facing if
   needed (from hip heading) — but the fixed +90° front-align is enough while the
   performer faces the screen.
3. RECORD state: keyframe the pose bones to an Action each tick; advance frame.
4. Stop button + N-panel UI (Start / Stop / Calibrate / Record).
5. (Later) IVRInput for the trigger if beep-driven recording proves insufficient.
