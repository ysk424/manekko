"""Apply a solved MuJoCo configuration back onto the Blender CC_Base armature.

The MJCF is built in Blender WORLD meters with body frames world-axis-aligned
at the A-pose rest (see rig.py). So at rest each MuJoCo body world rotation is
identity, and at runtime body world rotation R(t) IS the world-space delta the
corresponding bone must undergo:

    bone_world(t) = [ R(t) @ rest_world_rot | xpos(t) ]

pose_bone.matrix is in armature/object space, so we map back through
arm.matrix_world.inverted() (which also rescales meters→object units).
Uniform object scale (0.01) leaves rotation unaffected.

Bones are set in hierarchy (root-first) order with a view-layer update so each
child sees its parent's updated pose.
"""
from __future__ import annotations

import mathutils
import mujoco


def _world_rot(mat4: mathutils.Matrix) -> mathutils.Matrix:
    """Pure 3x3 rotation (scale removed) as a 4x4."""
    return mat4.to_quaternion().to_matrix().to_4x4()


def apply_pose(arm, rm, configuration, *, view_layer=None) -> None:
    """Set CC_Base pose bones from the solved MuJoCo configuration.

    ``pose_bone.matrix`` is absolute (armature space), so values are computed
    purely from MuJoCo — skipped intermediate bones (e.g. NeckTwist02) simply
    stay at rest without introducing error. But Blender derives each bone's
    ``matrix_basis`` from its *current* parent pose, so we must finalize a
    parent before its children: set in root-first order, updating after each.
    """
    import bpy
    if view_layer is None:
        view_layer = bpy.context.view_layer

    model = configuration.model
    data = configuration.data
    mw = arm.matrix_world
    mw_inv = mw.inverted()
    pbones = arm.pose.bones

    for body in rm.bodies:  # CHAIN order = root-first
        bone = rm.body_to_bone[body]
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)

        xpos = data.xpos[bid]
        xmat = data.xmat[bid]  # flat 9
        Rb = mathutils.Matrix((
            (xmat[0], xmat[1], xmat[2]),
            (xmat[3], xmat[4], xmat[5]),
            (xmat[6], xmat[7], xmat[8]),
        )).to_4x4()

        rest_rot = _world_rot(mw @ arm.data.bones[bone].matrix_local)
        world = mathutils.Matrix.Translation(
            (xpos[0], xpos[1], xpos[2])
        ) @ (Rb @ rest_rot)

        pbones[bone].matrix = mw_inv @ world
        view_layer.update()  # finalize this bone before its children are set
