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


def apply_pose(arm, rm, configuration, *, view_layer=None) -> None:
    import bpy
    if view_layer is None:
        view_layer = bpy.context.view_layer

    model = configuration.model
    data = configuration.data
    qpos = data.qpos
    mw_inv = arm.matrix_world.inverted()
    pbones = arm.pose.bones

    for body in rm.bodies:
        bone = rm.body_to_bone[body]
        pb = pbones[bone]
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)

        if model.body_jntnum[bid] == 0:
            # weld: no DOF — follows its parent rigidly, so its local
            # displacement from rest is zero. Reset the basis (clears any stale
            # pose) rather than skipping; a leftover rotation here would
            # otherwise propagate down the whole chain (e.g. clavicle -> arm).
            pb.matrix_basis = mathutils.Matrix.Identity(4)
            continue

        jid = int(model.body_jntadr[bid])
        jtype = model.jnt_type[jid]
        adr = int(model.jnt_qposadr[jid])

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
            axis = mathutils.Vector((
                model.jnt_axis[jid][0], model.jnt_axis[jid][1], model.jnt_axis[jid][2]))
            pb.rotation_mode = "XYZ"
            pb.rotation_euler = mathutils.Quaternion(axis, angle).to_euler("XYZ")

    view_layer.update()
