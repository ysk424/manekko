"""Apply a solved MuJoCo configuration onto the Blender CC_Base armature.

**Retargeting principle (do not regress):** the only data pushed onto the
character's bones is *FK joint rotation* — each joint's displacement from the
rest (A) pose — except the root, which additionally receives a global position.
World matrices / positions / scales are never baked onto non-root bones,
because the mocap performer and the character differ in size: only joint
angles transfer cleanly across body proportions.

This works because ``rig.py`` builds the MuJoCo model so that

* the rest configuration is all-identity joints (rest qpos = identity), and
* each body frame coincides with its Blender bone's local rest frame.

So a joint's ``qpos`` *is* the bone-local rotation away from rest:

* ball joint  -> a quaternion in the bone frame      -> ``rotation_euler``
* hinge joint -> an angle about the bone-frame axis  -> ``rotation_euler``
* free root   -> world pos + world quat              -> root bone global pose
* weld        -> no DOF                               -> stays at rest

Because every non-root bone gets only a *local* rotation (relative to its
parent's rest), the values are independent of one another, so no per-bone
``view_layer.update`` is needed — one update at the end suffices.
"""
from __future__ import annotations

import mathutils
import mujoco


# Stage-2 retarget knob (SCALAR, no angles): where to place the (performer-FK)
# root height on the character. With the performer-sized model (P1) the root
# already solves at ~the character's hip height, so 1.0 keeps it as-is. If the
# character floats or you want it lower, drop this (scales the root world height
# about the floor Z=0). This is the correct side of the Stage-1/Stage-2 seam:
# it touches only the character's root placement, never the performer-FK solve.
ROOT_HEIGHT_SCALE = 1.0


# --- Wrist 2-axis from the trackpad (v0.5.0) -------------------------------
# Fingers are NO LONGER driven here: the sibling app *manecam* (AI-server finger
# capture) owns the fingers, so manekko stops sending any finger angles and the
# controller trigger is unused. Instead the controller TRACKPAD drives the WRIST.
#
# The wrist is NOT an IK target and the Hand body stays a WELD in the MuJoCo rig
# (rig.CHAIN). We drive the Hand BONE directly: after the solve, apply a 2-axis
# rotation to the Hand pose bone. The wrist has two DOF, defined from the palm-down
# rest pose:
#   * FLEX      = up/down  (palmar/dorsi flexion)
#   * DEVIATION = inner/outer, toward/away from the BODY CENTER (radial/ulnar)
#
# IMPORTANT: the pad geometry (brain<->hardware swap) and the L/R deviation mirror
# are done at ACQUISITION (ops._wrist_from_ctrl), which hands us (flex, dev) ALREADY
# in wrist semantics (each -1..1, 0 = rest, + = up / + = inner). So this function
# only maps those two scalars onto the Hand bone's local rotation axes. The two
# LOCAL axes (which Hand-bone axis is flex vs deviation) and the global up/inner
# sense are rig-specific -> TUNE LIVE: flip WRIST_FLEX_AXIS/WRIST_FLEX_SIGN if
# up/down is inverted, WRIST_DEV_AXIS/WRIST_DEV_SIGN if inner/outer is inverted
# (these flip BOTH hands together; the per-hand mirror lives in ops). Same
# "experiment & adjust" approach as the elbow / the old finger curl.
WRIST_BONE = {"hand_l": "CC_Base_L_Hand", "hand_r": "CC_Base_R_Hand"}
WRIST_RANGE_DEG = 45.0           # deflection (deg) at full pad (|component| = 1)
WRIST_FLEX_AXIS = (1.0, 0.0, 0.0)   # Hand-local axis for up(+)/down(-) flexion
WRIST_DEV_AXIS = (0.0, 0.0, 1.0)    # Hand-local axis for inner(+)/outer(-) deviation
WRIST_FLEX_SIGN = +1.0              # flip if up/down inverted on hardware (both hands)
WRIST_DEV_SIGN = +1.0               # flip if inner/outer inverted on hardware (both hands)


def _apply_wrist(arm, wrist) -> None:
    """wrist: hand role -> (flex, dev), each -1..1 (0 = rest). The brain<->pad
    swap and the L/R mirror were already applied at acquisition
    (ops._wrist_from_ctrl); here flex rotates the Hand bone about WRIST_FLEX_AXIS
    and dev about WRIST_DEV_AXIS, by value * WRIST_RANGE_DEG, with the global
    per-DOF sign knobs (to fix a wrong-way bone axis). Only hands present in
    `wrist` are touched; absent hands keep the rest pose set by the weld branch of
    apply_pose."""
    import math
    import mathutils
    pbones = arm.pose.bones
    rng = math.radians(WRIST_RANGE_DEG)
    flex_axis = mathutils.Vector(WRIST_FLEX_AXIS)
    dev_axis = mathutils.Vector(WRIST_DEV_AXIS)
    for role, val in wrist.items():
        bone = WRIST_BONE.get(role)
        if bone is None:
            continue
        pb = pbones.get(bone)
        if pb is None:
            continue
        flex = WRIST_FLEX_SIGN * float(val[0])   # up(+)/down(-), already wrist-semantic
        dev = WRIST_DEV_SIGN * float(val[1])     # inner(+)/outer(-), already wrist-semantic
        q = (mathutils.Quaternion(flex_axis, rng * flex)
             @ mathutils.Quaternion(dev_axis, rng * dev))
        pb.rotation_mode = "XYZ"
        pb.rotation_euler = q.to_euler("XYZ")


def _apply_body_table(rm, model):
    """Precompute the per-body data the per-frame apply loop needs, so each
    tick avoids ``mj_name2id`` + the MuJoCo model array reads (body_jntnum /
    body_jntadr / jnt_type / jnt_qposadr / jnt_axis). Built once and cached on
    ``rm`` (``._apply_table``); rm and its model are created together in
    LiveDriver.__init__ and never rebuilt independently, so it can't go stale.

    Each entry is ``(bone_name, jtype, qpos_adr, hinge_axis)`` where ``jtype``
    is ``None`` for a weld (no DOF) and ``hinge_axis`` is a precomputed
    ``mathutils.Vector`` for hinges (``None`` otherwise)."""
    table = []
    for body in rm.bodies:
        bone = rm.body_to_bone[body]
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
        if model.body_jntnum[bid] == 0:
            table.append((bone, None, 0, None))   # weld: no DOF
            continue
        jid = int(model.body_jntadr[bid])
        jtype = model.jnt_type[jid]
        adr = int(model.jnt_qposadr[jid])
        axis = None
        if jtype == mujoco.mjtJoint.mjJNT_HINGE:
            axis = mathutils.Vector((
                float(model.jnt_axis[jid][0]),
                float(model.jnt_axis[jid][1]),
                float(model.jnt_axis[jid][2])))
        table.append((bone, jtype, adr, axis))
    return table


def apply_pose(arm, rm, configuration, *, wrist=None, view_layer=None) -> None:
    import bpy
    if view_layer is None:
        view_layer = bpy.context.view_layer

    model = configuration.model
    data = configuration.data
    qpos = data.qpos
    mw_inv = arm.matrix_world.inverted()
    pbones = arm.pose.bones

    table = getattr(rm, "_apply_table", None)
    if table is None:
        table = _apply_body_table(rm, model)
        rm._apply_table = table

    for bone, jtype, adr, axis in table:
        pb = pbones[bone]

        if jtype is None:
            # weld: no DOF — follows its parent rigidly, so its local
            # displacement from rest is zero. Reset the basis (clears any stale
            # pose) rather than skipping; a leftover rotation here would
            # otherwise propagate down the whole chain (e.g. clavicle -> arm).
            pb.matrix_basis = mathutils.Matrix.Identity(4)
            continue

        if jtype == mujoco.mjtJoint.mjJNT_FREE:
            # Root: the one allowed position transfer. Set the bone's GLOBAL
            # pose (position + orientation). Scale is explicitly stripped so
            # the armature object's inverse scale never contaminates the bone.
            pos = mathutils.Vector((qpos[adr], qpos[adr + 1], qpos[adr + 2]))
            pos.z *= ROOT_HEIGHT_SCALE   # Stage-2: character root height placement
            quat = mathutils.Quaternion(
                (qpos[adr + 3], qpos[adr + 4], qpos[adr + 5], qpos[adr + 6]))
            world = mathutils.Matrix.Translation(pos) @ quat.to_matrix().to_4x4()
            loc, rot, _ = (mw_inv @ world).decompose()
            # set the absolute armature-space pose (root has no parent bone);
            # scale forced to 1 so the object's inverse scale can't leak in.
            pb.matrix = mathutils.Matrix.LocRotScale(
                loc, rot, mathutils.Vector((1.0, 1.0, 1.0)))
        elif jtype == mujoco.mjtJoint.mjJNT_BALL:
            q = mathutils.Quaternion(
                (qpos[adr], qpos[adr + 1], qpos[adr + 2], qpos[adr + 3]))
            pb.rotation_mode = "XYZ"
            pb.rotation_euler = q.to_euler("XYZ")
        elif jtype == mujoco.mjtJoint.mjJNT_HINGE:
            angle = float(qpos[adr])
            pb.rotation_mode = "XYZ"
            pb.rotation_euler = mathutils.Quaternion(axis, angle).to_euler("XYZ")

    if wrist:
        _apply_wrist(arm, wrist)

    view_layer.update()
