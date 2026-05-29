"""Reduced CC_Base skeleton → MuJoCo MJCF for mink full-body IK.

The CC4 ``CC_Base`` armature has 101 bones (twists, fingers, toes, face).
For 10-tracker full-body IK we only need the main chain. This module
extracts that chain from the live Blender armature (in metric world space)
and emits an MJCF whose bodies are world-axis-aligned at the rest (A) pose,
so the rest configuration is all-identity joint rotations.

Tracker roles (10): head, hip, hand_l/r, foot_l/r, elbow_l/r, knee_l/r.
Each maps to a MuJoCo body that mink drives with a FrameTask.

Units: Blender armature object scale is 0.01, scene is metric → world-space
bone coordinates (arm.matrix_world @ head_local) are already in meters,
matching SteamVR. We build the MJCF directly in those meters.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import mathutils


# (bone_name, model_parent_or_None, joint_type)
#   joint_type: "free" | "ball" | "hinge" | "weld"
CHAIN: list[tuple[str, str | None, str]] = [
    ("CC_Base_Hip",         None,                  "free"),
    ("CC_Base_Waist",       "CC_Base_Hip",         "ball"),
    ("CC_Base_Spine01",     "CC_Base_Waist",       "ball"),
    ("CC_Base_Spine02",     "CC_Base_Spine01",     "ball"),
    ("CC_Base_NeckTwist01", "CC_Base_Spine02",     "ball"),
    ("CC_Base_Head",        "CC_Base_NeckTwist01", "ball"),
    # spine→pelvis→legs
    ("CC_Base_Pelvis",      "CC_Base_Hip",         "weld"),
    ("CC_Base_L_Thigh",     "CC_Base_Pelvis",      "ball"),
    ("CC_Base_L_Calf",      "CC_Base_L_Thigh",     "hinge"),
    ("CC_Base_L_Foot",      "CC_Base_L_Calf",      "ball"),
    ("CC_Base_R_Thigh",     "CC_Base_Pelvis",      "ball"),
    ("CC_Base_R_Calf",      "CC_Base_R_Thigh",     "hinge"),
    ("CC_Base_R_Foot",      "CC_Base_R_Calf",      "ball"),
    # spine→arms (clavicle welded to keep shoulder stable in v1)
    ("CC_Base_L_Clavicle",  "CC_Base_Spine02",     "weld"),
    ("CC_Base_L_Upperarm",  "CC_Base_L_Clavicle",  "ball"),
    ("CC_Base_L_Forearm",   "CC_Base_L_Upperarm",  "hinge"),
    ("CC_Base_L_Hand",      "CC_Base_L_Forearm",   "ball"),
    ("CC_Base_R_Clavicle",  "CC_Base_Spine02",     "weld"),
    ("CC_Base_R_Upperarm",  "CC_Base_R_Clavicle",  "ball"),
    ("CC_Base_R_Forearm",   "CC_Base_R_Upperarm",  "hinge"),
    ("CC_Base_R_Hand",      "CC_Base_R_Forearm",   "ball"),
]

# tracker role -> bone whose body frame the FrameTask targets
TRACKER_TO_BONE: dict[str, str] = {
    "hip":     "CC_Base_Hip",
    "head":    "CC_Base_Head",
    "hand_l":  "CC_Base_L_Hand",
    "hand_r":  "CC_Base_R_Hand",
    "foot_l":  "CC_Base_L_Foot",
    "foot_r":  "CC_Base_R_Foot",
    "elbow_l": "CC_Base_L_Forearm",
    "elbow_r": "CC_Base_R_Forearm",
    "knee_l":  "CC_Base_L_Calf",
    "knee_r":  "CC_Base_R_Calf",
}

# hinge axes (elbow/knee) are derived from the rest geometry per-bone below.


@dataclass
class RigModel:
    mjcf: str
    body_to_bone: dict[str, str]          # mjcf body name -> blender bone name
    bone_to_body: dict[str, str]
    tracker_to_body: dict[str, str]       # role -> mjcf body name
    rest_world: dict[str, mathutils.Matrix]  # bone -> armature/world rest matrix
    bodies: list[str] = field(default_factory=list)


def _mjcf_name(bone: str) -> str:
    # MuJoCo names are fine with these chars, but keep them tidy.
    return bone.replace("CC_Base_", "")


def _world_head(arm, bone_name: str) -> mathutils.Vector:
    b = arm.data.bones[bone_name]
    return arm.matrix_world @ b.head_local


def _world_tail(arm, bone_name: str) -> mathutils.Vector:
    b = arm.data.bones[bone_name]
    return arm.matrix_world @ b.tail_local


def build_mjcf(arm) -> RigModel:
    """Build an MJCF string + mappings from a Blender CC_Base armature object."""
    name_of = {bone: _mjcf_name(bone) for bone, _, _ in CHAIN}
    parent_of = {bone: parent for bone, parent, _ in CHAIN}
    joint_of = {bone: jt for bone, _, jt in CHAIN}

    # children index for tree emission
    children: dict[str | None, list[str]] = {}
    for bone, parent, _ in CHAIN:
        children.setdefault(parent, []).append(bone)

    body_to_bone: dict[str, str] = {}
    rest_world: dict[str, mathutils.Matrix] = {}
    for bone, _, _ in CHAIN:
        body_to_bone[name_of[bone]] = bone
        rest_world[bone] = arm.matrix_world @ arm.data.bones[bone].matrix_local

    def hinge_axis(bone: str) -> mathutils.Vector:
        """Flexion axis for elbow/knee = perpendicular to the two segment dirs."""
        parent = parent_of[bone]
        d_parent = (_world_tail(arm, parent) - _world_head(arm, parent)).normalized()
        d_self = (_world_tail(arm, bone) - _world_head(arm, bone)).normalized()
        axis = d_parent.cross(d_self)
        if axis.length < 1e-4:          # nearly straight (typical A-pose) → use a fallback
            # use cross of segment with world up as a stable lateral axis
            axis = d_self.cross(mathutils.Vector((0.0, 0.0, 1.0)))
        return axis.normalized()

    def emit_body(bone: str, parent: str | None) -> ET.Element:
        bname = name_of[bone]
        head_w = _world_head(arm, bone)
        if parent is None:
            pos = head_w
        else:
            pos = head_w - _world_head(arm, parent)
        body = ET.Element("body", name=bname, pos=_v(pos))

        jt = joint_of[bone]
        if jt == "free":
            ET.SubElement(body, "freejoint", name=f"{bname}_free")
        elif jt == "ball":
            ET.SubElement(body, "joint", name=bname, type="ball", damping="1")
        elif jt == "hinge":
            ax = hinge_axis(bone)
            ET.SubElement(body, "joint", name=bname, type="hinge",
                          axis=_v(ax), damping="1", limited="false")
        elif jt == "weld":
            pass  # no joint: rigid offset from parent

        # capsule geom (gives inertia; also a visual). fromto in body frame
        # (world-aligned) = tail - head.
        seg = _world_tail(arm, bone) - head_w
        if seg.length > 1e-3:
            ET.SubElement(body, "geom", type="capsule",
                          fromto=f"0 0 0 {seg.x:.5f} {seg.y:.5f} {seg.z:.5f}",
                          size="0.025")
        else:
            ET.SubElement(body, "geom", type="sphere", size="0.04")

        for child in children.get(bone, []):
            body.append(emit_body(child, bone))
        return body

    mujoco = ET.Element("mujoco", model="cc_base")
    ET.SubElement(mujoco, "compiler", angle="radian", autolimits="true")
    ET.SubElement(mujoco, "option", timestep="0.01")
    default = ET.SubElement(mujoco, "default")
    ET.SubElement(default, "geom", rgba="0.7 0.7 0.8 1")
    worldbody = ET.SubElement(mujoco, "worldbody")
    # single root chain (Hip)
    for root in children.get(None, []):
        worldbody.append(emit_body(root, None))

    ET.indent(mujoco, space="  ")
    mjcf = ET.tostring(mujoco, encoding="unicode")

    return RigModel(
        mjcf=mjcf,
        body_to_bone=body_to_bone,
        bone_to_body={v: k for k, v in body_to_bone.items()},
        tracker_to_body={role: name_of[bone] for role, bone in TRACKER_TO_BONE.items()},
        rest_world=rest_world,
        bodies=[name_of[b] for b, _, _ in CHAIN],
    )


def _v(vec) -> str:
    return f"{vec[0]:.6f} {vec[1]:.6f} {vec[2]:.6f}"
