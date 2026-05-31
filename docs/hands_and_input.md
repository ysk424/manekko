# Hands & controller input (v0.1.1–0.1.5)

How the hands work, how controller input is read, and the wheel/bootstrap note.
Production path is `src/ops.py` → `LiveDriver` (see [the live-path note in
CLAUDE.md]); the `TrackerReader` thread is dev-only.

## Wrist (hand) orientation

The two former elbow trackers were moved onto the **wrists**; they drive the
`hand_l/r` IK targets (`CC_Base_*_Hand`), position **and** orientation (v0.1.0
rotation rollout). So wrist twist/orientation follows the wrist tracker. The
controllers are no longer the hand IK target — they're in the palms for input.

## Controller input: legacy works under the Overlay app type

The big finding (v0.1.1 spike): the **legacy** `IVRSystem.getControllerState`
returns no live data under `VRApplication_Background` (packet number frozen), but
**works under `VRApplication_Overlay`** — trigger, trackpad and buttons are all
live, poses still work. So `ops._init_vr` inits **Overlay** (falls back to
Background if Overlay init fails); `OPENVR_APP_TYPE` reverts it.

Verified button masks (VIVE wand, `1<<id`): trigger `0x2_0000_0000` (bit 33,
SteamVR_Trigger), trackpad `0x1_0000_0000` (bit 32, SteamVR_Touchpad), menu
`0x2` (ApplicationMenu), grip/side `0x4` (Grip). Analog: `rAxis[1].x` = trigger
(0..1), `rAxis[0].(x,y)` = trackpad. (IVRInput, the modern API, was investigated
earlier and the binding never went `bActive` for a background app; legacy +
Overlay is what we use.)

## Finger grip/extend

`ops._grip_from_ctrl` maps each palm controller's trigger (`rAxis[1].x`, 0..1) to
the same-side hand. `apply._apply_finger_curl` curls every finger bone (all
descendants of `CC_Base_*_Hand`, found naming-agnostically by
`apply.finger_bone_names`) from rest (open, t=0) toward a fist (t=1).

The curl axis lies in the bone-local **X-Z plane** (perpendicular to the bone
length / local Y) at a tuned angle → `axis = (cos, 0, sin)`; rotate by
`t * FINGER_CURL_ANGLE`. The flexion axis can't be auto-derived from straight
rest fingers (degenerate cross-product), so the direction is a tuned scalar.

Tuned live (a temporary N-panel field, since removed) by sweeping 0–360° and
watching each group; the values are baked in `apply.FINGER_CURL_DIR_DEG`:

| hand | thumb | other 4 fingers |
|---|---|---|
| L | 200° | 270° |
| R | 160° | 90° |

L/R mirror exactly (R = 360 − L per group). Thumb differs from the other four.
To re-tune for a different rig, change these four numbers. Fingers are NOT part
of the IK; they're applied directly in `apply_pose` and keyframed on Record.

## Wheel bootstrap & the sys.path policy warning

The extension self-bootstraps its bundled wheels: `__init__._ensure_wheels()`
extracts `./wheels/*.whl` into `./_libs` once and prepends it to `sys.path`
(no-op if the deps already import). This is **required here** — with the manifest
`wheels=[...]` alone, `import openvr` raises `ModuleNotFoundError` (verified on
v0.1.3, which removed the bootstrap). Blender's own wheel mechanism does not
surface the deps in this environment.

Blender 5.1 flags the `sys.path` insert as **"Policy violation with sys.path:
._libs"**. This warning is **cosmetic**: it does not block the extension, and it
is **not** why the Add-ons UI lacks an uninstall button (removing the bootstrap
in v0.1.3 cleared the warning but the uninstall button was still absent — they're
unrelated). **Uninstall by deleting the installed extension folder**
(`.../extensions/user_default/manekko/`).
