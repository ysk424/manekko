# Manekko

> Real-time full-body IK mocap in Blender from **SteamVR + VIVE Tracker 3.0**,
> retargeted onto a **Character Creator (CC)** character. Live drive + bake to an Action.

---

## Status

Live full-body mocap works end to end: **trackers → mink IK → CC bones, real-time in the
viewport, with record-to-Action**. IK tuning and finer calibration are still in progress.

**Treat this as a reference implementation for one specific rig — not turnkey software.**
It is hardcoded to the author's hardware, room and character. You are expected to change it.

## Why this is unusual

The combination **VIVE Tracker 3.0 + SteamVR + Blender full-body IK onto a CC character** is
rare and there are almost no public examples. The genuinely valuable part of this repo is the
**map of landmines** we hit and solved — see the docs below. If this saves someone else the
same week of pain, it did its job.

## Adapt it to YOUR rig (important)

This will **not** run as-is. To use it, edit:

- **`src/openvr_reader.py` → `SERIAL_TO_ROLE`** — the author's tracker/controller serials mapped
  to roles. Replace with *your* device serials (lay them out on the floor in a line and read the
  positions to identify them — that's how these were assigned).
- **`src/openvr_reader.py` → `world_to_blender` / `FRONT_YAW_DEG`** — front alignment. The author's
  screen is at room **−X** and the performer faces it, so a fixed **+90° yaw** maps "facing the
  screen" to the character's front (Blender −Y). Net transform: SteamVR `(x,y,z) → (z,x,y)`. Change
  this for your room, or derive the yaw from the hip-tracker heading.
- **Character** — built for CC4 `CC_Base` armatures; auto-detected via the `CC_Base_Hip` bone.
- **`CLAUDE.md`** holds the author's machine paths and working log; it is the dev journal, not
  documentation.

## Built with Claude Code

This codebase **and its docs were written with [Claude Code](https://claude.com/claude-code)
(Anthropic)**, interactively. The hard parts were worked out in that loop:

- FK-angle-only retargeting (send joint *angles*, never world matrices — performer and character
  differ in size),
- the SteamVR **IVRInput** background-app binding investigation (and why the legacy controller API
  is dead),
- the **null-space drift** diagnosis and fix (cyclic motion ratchets the body into a twist →
  raise `posture_cost`),
- the coordinate / front-alignment transforms.

In 2026 an indie doesn't have to hand-write all of this. The fastest way to adapt this repo to
**your** trackers / room / character is to point Claude Code at it and iterate — the docs below
are written to be exactly that kind of starting map.

## The valuable docs (read these first)

- **[`docs/mink_pitfalls.md`](docs/mink_pitfalls.md)** — differential-IK landmines and fixes:
  null-space drift, the A-pose straight-limb singularity, missing joint limits, quaternion/units
  gotchas.
- **[`docs/live_driving_notes.md`](docs/live_driving_notes.md)** — live-driving findings: why a
  blocking loop never shows on screen (use a modal + timer), the coordinate transform, the
  trigger/IVRInput status, and a validated modal prototype.

(Both docs are written in Japanese; point Claude Code at them to translate or adapt.)

## Architecture

| File | Role |
|---|---|
| `src/rig.py` | CC_Base armature → reduced MuJoCo MJCF (each body frame aligned to its bone's local rest frame, so a joint's `qpos` *is* the bone-local rotation from rest). |
| `src/solver.py` | mink differential IK: 10 position-only `FrameTask`s + a `PostureTask` (daqp). `posture_cost=1e-1` to kill null-space drift. |
| `src/apply.py` | **FK-angle-only** retarget: joint `qpos` → bone Euler; the root alone gets a global position. No world matrices/scale are baked onto bones (size-independent). |
| `src/openvr_reader.py` | OpenVR pose reader (off the main thread), coordinate transform, `Calibration`. |
| `src/live.py` | `LiveDriver`: snapshot → calibrate → solve → apply. |
| `src/ops.py`, `ui.py` | N-panel **Manekko**: Start/Stop, Record, Calibrate (beep-driven, 5 s countdowns). |
| `src/calibration_io.py` | Save/load the A-pose calibration to the user config dir (auto-loaded on Start). |

## Hardware / requirements

- **Blender 5.1** (Python 3.13, win-amd64).
- **SteamVR** + VIVE Tracker 3.0 (×8: head, hip, elbows, knees, feet) + **2 VIVE controllers**
  (the hands) + lighthouses + an HMD (reference only).
- Dependencies (`mink`, `mujoco`, `openvr`, `qpsolvers`/`daqp`, `scipy`, …) ship as **bundled
  wheels** in the extension; `numpy` comes from Blender itself.

## Install / build

```
blender --command extension build --source-dir . --output-dir dist
```

Then in Blender: **Preferences → Get Extensions → ▼ → Install from Disk →** the built `.zip`.
Open the **Manekko** tab in the 3D View sidebar (`N`). Buttons: **Start/Stop**, **Record (5s)**,
**Calibrate (5s)**. First run: Start → Calibrate (hold the A-pose for 5 s) → it drives and saves
the calibration; it auto-loads next time.

## License

Apache-2.0.
