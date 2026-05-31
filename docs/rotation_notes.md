# Rotation (orientation tracking) — design & triage

Added in v0.0.10 (head + hip) and expanded to all IK roles in v0.0.11 / shipped
as **v0.1.0**. Position-only IK was the v1; this doc covers how orientation was
added without falling into the "rotation is a minefield" traps the owner warned
about.

## Why it's tractable here: rotation is handled on the PERFORMER side

The hard part of rotation in mocap is the coordinate-frame zoo (SteamVR world,
Blender world, character bone rest frames, the room↔character yaw). Doing the
math in Blender's world frame means chaining several conversions and getting any
one of them wrong flips/spins the body.

We avoid that by registering, at the A-pose, a **mount offset in the tracker's
own (local) frame**, exactly mirroring how the *position* calibration absorbs
"where the performer stands":

- Position: `offset = body_rest_pos − raw_tracker_pos`; live `target = raw + offset`.
- Orientation: `M = R_raw_apose⁻¹ · R_rest`; live `target = R_raw_live · M`.

`M` is a constant rotation in the tracker-local frame (the physical mounting of
the tracker on the bone). At the A-pose the target reproduces the body's rest
orientation exactly; as the performer rotates by any world delta `dR`
(`R_raw_live = dR · R_raw_apose`), the target becomes `dR · R_rest` — i.e. the
body turns by the *same world rotation as the tracker*. The room↔character yaw
and the tracker mounting are both swallowed by `M`, so **no separate
Blender-world rotation conversion is needed**.

## Pipeline (where each piece lives)

1. `openvr_reader.world_rot_to_blender(R)` = `W @ R`, where `W` is the same axis
   map as `world_to_blender` for positions (svr `(x,y,z) → (z,x,y)`, a proper
   rotation, det +1). Keeps orientation and position in the same Blender frame.
   `R` is the device's local→world rotation from `_mat34_rot` (OpenVR 3×4).
2. `ops._read_valid` returns `(pos_snap, rot_snap)` per role (production live
   path — NOT the unused `TrackerReader` thread; see CLAUDE.md).
3. `Calibration.from_apose(..., raw_rotations, body_rest_rotations)` computes
   `rot_offset[role] = R_raw_apose.T @ R_rest`. `R_rest` = body world orientation
   at qpos0 (`configuration.data.xmat[bid].reshape(3,3)`, via
   `LiveDriver.body_rest_orientations`). Roles with no rotation data stay
   position-only.
4. `Calibration.apply_rot(raw_rotations)` → `{role: R_raw_live @ rot_offset}`,
   only for roles that have a registered `rot_offset`.
5. `solver.set_target_poses(positions, orientations)` builds the FrameTask SE3
   target (`SE3.from_rotation_and_translation(SO3.from_matrix(R), pos)`); a role
   with no orientation gets identity rotation (ignored when its
   `orientation_cost` is 0).
6. `solver` sets `orientation_cost` (default `1e-1`) only for `orientation_roles`
   (default = all 7 IK roles). Drop a role from that tuple to make it
   position-only again.
7. `apply.py` is **unchanged**: the hip is the free root (already gets world
   pos + quat), the head/chest/hands/feet are ball/hinge joints (already get
   `rotation_euler` from qpos). Orienting the bodies in the solve flows through
   automatically.
8. `calibration_io` persists `rot_offset` (3×3 lists) to the CONFIG JSON;
   backward compatible (old files without it load position-only).

## Validated offline (against the real code)

`Calibration.from_apose` → `apply_rot` round-trip with random orthonormal
matrices: A-pose target reproduces `R_rest` to ~1e-16; a world delta `dR` yields
`dR @ R_rest` to ~1e-16; targets stay orthonormal (det +1, so `SO3.from_matrix`
is safe); `world_rot_to_blender` matches `world_to_blender` on the basis axes.

## Live status (v0.1.0)

Owner PASS: rotation works — straightened-leg twist tracks; arms can be twisted
in T- or A-pose. Whether the orientation is *strongly* enough applied is not yet
confirmed numerically; it will be obvious once the palm/hand input is wired.

## Triage when a part misbehaves

- **A whole part spins / keeps rotating** → bad registration for that role
  (re-Calibrate cleanly in the A-pose) or a frame/sign issue. Note *which way*
  it spins. Quick isolation: remove that role from `solver.orientation_roles`
  (it falls back to position-only) and rebuild.
- **Twisting limbs go weird under load** → expected risk (the staged-rotation
  principle says twisting limbs are the riskiest). Drop `hand_l/r` and/or
  `foot_l/r` from `orientation_roles` first.
- **Tracking too weak / laggy** → raise `solver.orientation_cost` (try 0.3–0.5;
  position_cost is 1.0, posture_cost is 1e-1).
- **Tracking fights position** → lower `orientation_cost`.

## Knobs

`solver.orientation_roles`, `solver.orientation_cost`. Everything else about
rotation is registration (automatic at Calibrate) — no per-frame angle math to
hand-tune.
