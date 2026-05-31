# Manekko

> Real-time full-body IK mocap in Blender from **SteamVR + VIVE Tracker 3.0**,
> retargeted onto a **Character Creator (CC)** character. Live drive + bake to an Action.

---

## Status

**v0.1.5 — first complete product.** Live full-body mocap works end to end:
**trackers → mink IK → CC bones, real-time in the viewport, with record-to-Action.**

- **Position + orientation** for all 7 IK targets (hip, head, chest, 2 wrists, 2 feet).
- **Finger open/close** driven by the controller trigger (analog 0..1 → fist).
- Face capture is **out of scope** here — use iPhone ARKit as a separate project.

**Treat this as a reference implementation for one specific rig — not turnkey software.**
It is hardcoded to the author's hardware, room and character. You are expected to change it.

## Why this is unusual

The combination **VIVE Tracker 3.0 + SteamVR + Blender full-body IK onto a CC character** is
rare and there are almost no public examples. The genuinely valuable part of this repo is the
**map of landmines** we hit and solved — see the docs below. If this saves someone else the
same week of pain, it did its job.

## How it works (the short version)

1. **Performer-sized model.** `rig.py` builds a reduced MuJoCo model whose segment lengths are
   the *performer's* (not the character's), so the solved joint angles are the performer's true
   angles — no character size leaks into the solve.
2. **Position + orientation IK.** `solver.py` runs mink differential IK (daqp): a `FrameTask` per
   tracker plus a `PostureTask` anchoring to the A-pose.
3. **FK-angle-only retarget.** `apply.py` sends only joint *angles* to the CC bones (the root also
   gets a global position). World matrices/scale are never baked — only angles transfer across the
   performer↔character size difference.
4. **Performer-side rotation registration.** Orientation is registered at the A-pose as a mount
   offset in the *tracker's own frame* (`M = R_raw_apose⁻¹·R_rest`; live target `= R_raw_live·M`),
   which absorbs the room↔character frame difference — no fragile Blender-world rotation math.
5. **Fingers from the trigger.** The controllers (in the palms) report an analog trigger; that
   0..1 drives a finger curl blend (open↔fist). Legacy controller input only works with the OpenVR
   **Overlay** app type — it's dead under Background.

## Adapt it to YOUR rig (important)

This will **not** run as-is. To use it, edit:

- **`src/openvr_reader.py` → `SERIAL_TO_ROLE`** — the author's tracker/controller serials mapped to
  roles (`hip, head, chest, hand_l/r` = wrists, `foot_l/r`, `palm_l/r` = controllers). Replace with
  *your* serials (lay the devices on the floor in a line and read positions to identify them).
- **`src/openvr_reader.py` → `world_to_blender` / `FRONT_YAW_DEG`** — front alignment. The author's
  screen is at room **−X** and the performer faces it, so a fixed **+90° yaw** maps "facing the
  screen" to the character's front (Blender −Y). Net transform: SteamVR `(x,y,z) → (z,x,y)`.
- **`src/openvr_reader.py` → `BONE_OFFSET_M`** — how far each tracker is pushed toward the bone
  along its normal (per-role cm).
- **`src/apply.py` → `FINGER_CURL_DIR_DEG`** — per-hand/per-thumb finger curl direction (degrees);
  rig-specific, tuned live.
- **`src/rig.py` → `PERFORMER_*` / `GLOBAL_HEIGHT_SCALE`** — your body segment lengths / height.
- **Character** — built for CC4 `CC_Base` armatures; auto-detected via the `CC_Base_Hip` bone.
- **`CLAUDE.md`** holds the author's machine paths and working log; it is the dev journal, not docs.

## Built with Claude Code

This codebase **and its docs were written with [Claude Code](https://claude.com/claude-code)
(Anthropic)**, interactively. The hard parts were worked out in that loop:

- FK-angle-only retargeting (send joint *angles*, never world matrices — performer and character
  differ in size),
- performer-sized solve + performer-side rotation registration (so adding orientation didn't turn
  into a coordinate-frame minefield),
- the SteamVR controller-input investigation: the **legacy** `getControllerState` is dead under the
  Background app type but **works under Overlay** (trigger/trackpad/buttons all live),
- the **null-space drift** diagnosis and fix (cyclic motion ratchets the body into a twist →
  raise `posture_cost`),
- the coordinate / front-alignment transforms, and that the bundled-wheel sys.path bootstrap is
  required here (Blender's own wheel mechanism does not surface the deps in this environment).

In 2026 an indie doesn't have to hand-write all of this. The fastest way to adapt this repo to
**your** trackers / room / character is to point Claude Code at it and iterate.

## The valuable docs (read these first)

- **[`docs/mink_pitfalls.md`](docs/mink_pitfalls.md)** — differential-IK landmines and fixes:
  null-space drift, the A-pose straight-limb singularity, missing joint limits, quaternion/units.
- **[`docs/live_driving_notes.md`](docs/live_driving_notes.md)** — live-driving findings: why a
  blocking loop never shows on screen (use a modal + timer), the coordinate transform, a validated
  modal prototype.
- **[`docs/rotation_notes.md`](docs/rotation_notes.md)** — adding orientation the performer-side way
  (A-pose mount-offset registration), pipeline, offline validation, and a triage guide.
- **[`docs/hands_and_input.md`](docs/hands_and_input.md)** — controller input under the Overlay app
  type, trigger → finger grip, the finger-curl direction tuning, and the wheel/bootstrap note.

(Docs are written in Japanese; point Claude Code at them to translate or adapt.)

## Architecture

| File | Role |
|---|---|
| `src/rig.py` | CC_Base armature → reduced MuJoCo MJCF, scaled to the **performer's** segment lengths; each body frame aligned to its bone's rest frame (so `qpos` *is* the bone-local rotation from rest). 7 IK target bodies. |
| `src/solver.py` | mink IK: a `FrameTask` per tracker (**position + orientation**, `orientation_cost` per role) + a `PostureTask` (daqp). `posture_cost=1e-1` kills null-space drift. |
| `src/apply.py` | **FK-angle-only** retarget: joint `qpos` → bone Euler; root alone gets a global position. Plus trigger-driven **finger curl** (`FINGER_CURL_DIR_DEG`). No world matrices/scale baked onto bones. |
| `src/openvr_reader.py` | OpenVR pose reader, coordinate + orientation transforms, per-role bone offset, `Calibration` (position offset + orientation mount offset). |
| `src/live.py` | `LiveDriver`: snapshot → calibrate → solve → apply (incl. fingers). |
| `src/ops.py`, `ui.py` | N-panel **Manekko**: Start/Stop, Record, Calibrate. Reads poses + controller trigger each tick (OpenVR **Overlay** app type). |
| `src/calibration_io.py` | Save/load the A-pose calibration (position + orientation) to the user config dir; auto-loaded on Start. |

## Hardware / requirements

- **Blender 5.1** (Python 3.13, win-amd64).
- **SteamVR** + **7 VIVE Tracker 3.0** (head, hip, chest, 2 wrists, 2 feet; +1 spare) + **2 VIVE
  controllers** (held in the palms, for the finger trigger) + lighthouses + an HMD (reference only).
- Dependencies (`mink`, `mujoco`, `openvr`, `qpsolvers`/`daqp`, `scipy`, …) ship as **bundled
  wheels**; `numpy` comes from Blender itself.

> Note: the extension extracts its bundled wheels into `./_libs` and adds it to `sys.path` because
> Blender's own wheel mechanism does not surface them in this environment. Blender 5.1 shows a
> cosmetic *"Policy violation with sys.path"* warning for this; it does not affect functionality.

## Install / build

```
blender --command extension build --source-dir . --output-dir dist
```

Then in Blender: **Preferences → Get Extensions → ▼ → Install from Disk →** the built `.zip`.
Open the **Manekko** tab in the 3D View sidebar (`N`). Buttons: **Start/Stop**, **Record (5s)**,
**Calibrate (5s)**. First run: Start → Calibrate (hold the A-pose 5 s) → it drives and saves the
calibration (position + orientation); it auto-loads next time. To uninstall, delete the installed
extension folder.

## License

Apache-2.0.
