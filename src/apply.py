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


# --- Finger grip/extend (v0.1.1) -------------------------------------------
# Fingers are NOT part of the IK chain; they are driven directly by the
# controller trigger (analog 0..1) as a curl blend: rest (open) at 0, fist at 1.
# Every finger descendant of the hand bone is curled about one local euler axis
# by trigger * FINGER_CURL_ANGLE. Axis/sign are rig-specific — if a hand opens
# backwards or splays instead of making a fist, flip the sign or change the axis
# (tune live; same "experiment & adjust" approach as the other knobs).
FINGER_CURL_ANGLE = 1.2          # radians at full grip (~69 deg per joint)
# Curl DIRECTION per hand and per finger group (degrees), found live on this rig
# by sweeping the (now removed) N-panel field and watching each group. The curl
# axis lies in the bone-local X-Z plane (perpendicular to the bone length, which
# is local Y) at this angle -> axis = (cos, 0, sin). The thumb curls differently
# from the other four fingers, and L/R mirror (R = 360 - L for each group). The
# flexion axis can't be auto-derived from straight rest fingers (degenerate),
# so these are tuned constants — change them to re-tune for a different rig.
FINGER_CURL_DIR_DEG = {
    "hand_l": {"thumb": 200.0, "other": 270.0},
    "hand_r": {"thumb": 160.0, "other": 90.0},
}


def finger_bone_names(arm) -> dict[str, list[str]]:
    """Per hand role -> list of finger bone names (all descendants of the hand
    bone, naming-agnostic so it survives CC finger-name variants). Cache once."""
    out: dict[str, list[str]] = {}
    for side, role in (("L", "hand_l"), ("R", "hand_r")):
        hb = arm.data.bones.get(f"CC_Base_{side}_Hand")
        out[role] = [b.name for b in hb.children_recursive] if hb else []
    return out


def _apply_finger_curl(arm, curls, finger_names) -> None:
    """curls: hand role -> trigger 0..1. Curls each finger bone about the local
    X-Z-plane axis at FINGER_CURL_DIR_DEG[role][group] by trigger *
    FINGER_CURL_ANGLE (0 = rest/open), where group is 'thumb' for thumb bones
    and 'other' for the rest. Only hands present in `curls` are touched."""
    import math
    import mathutils
    pbones = arm.pose.bones
    for role, t in curls.items():
        dirs = FINGER_CURL_DIR_DEG.get(role)
        if dirs is None:
            continue
        t = float(t)
        eul = {}
        for group, deg in dirs.items():
            th = math.radians(deg)
            axis = mathutils.Vector((math.cos(th), 0.0, math.sin(th)))
            eul[group] = mathutils.Quaternion(
                axis, t * FINGER_CURL_ANGLE).to_euler("XYZ")
        for bn in finger_names.get(role, ()):
            pb = pbones.get(bn)
            if pb is None:
                continue
            pb.rotation_mode = "XYZ"
            pb.rotation_euler = eul["thumb"] if "Thumb" in bn else eul["other"]


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


def apply_pose(arm, rm, configuration, *, fingers=None, finger_names=None,
               view_layer=None) -> None:
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

    if fingers and finger_names:
        _apply_finger_curl(arm, fingers, finger_names)

    view_layer.update()
