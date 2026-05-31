"""Reduced CC_Base skeleton → MuJoCo MJCF for mink full-body IK.

The CC4 ``CC_Base`` armature has 101 bones (twists, fingers, toes, face).
For 10-tracker full-body IK we only need the main chain. This module
extracts that chain from the live Blender armature (in metric world space)
and emits an MJCF whose bodies are world-axis-aligned at the rest (A) pose,
so the rest configuration is all-identity joint rotations.

IK target roles (v0.0.8): hip, head, hand_l/r (wrists), foot_l/r, chest.
Each maps to a MuJoCo body that mink drives with a position-only FrameTask.

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
    "chest":   "CC_Base_Spine02",   # tracker on the sternum (v0.0.8, "総合案その1")
    # elbow trackers DROPPED from the IK (v0.0.5, 2026-05-31). The arm is over-
    # constrained (Upperarm ball 3 + Forearm hinge 1 = 4 DOF vs elbow 3 + wrist 3
    # = 6); the elbow + the imperfect controller wrist target fought and pinned
    # the elbow. We let the wrist (hand) drive the arm and the PostureTask resolve
    # the elbow swivel toward the A-pose. The elbow trackers are still read by
    # openvr_reader (just not targeted) so re-enabling later = uncomment these:
    # "elbow_l": "CC_Base_L_Forearm",
    # "elbow_r": "CC_Base_R_Forearm",
    # knee trackers ALSO DROPPED from the IK (v0.0.7, 2026-05-31). After the
    # performer-sized model (v0.0.6) made the arm solve stable with the elbow
    # dropped, we drop the knees too: the legs are driven by the foot targets and
    # the PostureTask resolves the knee swivel toward the A-pose. The knee trackers
    # are still read by openvr_reader (just not targeted). Re-enable = uncomment:
    # RISK: under load / deep knee-bend the Posture may mis-guess knee direction —
    # if so, put a knee tracker back here.
    # "knee_l":  "CC_Base_L_Calf",
    # "knee_r":  "CC_Base_R_Calf",
}

# hinge axes (elbow/knee) are derived from the rest geometry per-bone below.


# --- P1: performer-sized model (build the PERFORMER's FK, not the character's) ---
# Stage-1 cleanliness: scale each character bone SEGMENT to the performer's
# measured length while keeping the character A-pose directions/orientations
# (no performer armature exists; A-pose directions are a valid proxy). Joint
# CENTERS (shoulder/hip placement) stay character ratios for now (the deferred
# "fulcrum" fix). rest_rot is untouched, so apply.py needs no change and the
# solved qpos becomes the performer's true joint angles -> no double conversion.
# Set the ratios to match the character (or PERFORMER_*≈character) to fall back
# to the pure character model. Measured 2026-05-31: performer arm
# (shoulder->wrist) 0.55 m, thigh (hip->knee) 0.47 m, stature 1.80 m vs this CC
# character ~1.59 m. Unmeasured bones (torso/spine/neck/shin/etc.) use the global
# height stretch.
PERFORMER_ARM_M = 0.55          # one side, shoulder->wrist (elbow split not needed)
PERFORMER_THIGH_M = 0.47        # one side, hip joint->knee
GLOBAL_HEIGHT_SCALE = 1.80 / 1.59


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
    rest_rot: dict[str, mathutils.Quaternion] = {}
    for bone, _, _ in CHAIN:
        body_to_bone[name_of[bone]] = bone
        rest_world[bone] = arm.matrix_world @ arm.data.bones[bone].matrix_local
        # pure rotation (object scale removed by to_quaternion)
        rest_rot[bone] = rest_world[bone].to_quaternion()

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

    # --- P1: performer segment scales. Default = global height stretch; the two
    # measured limbs are overridden. Scaling a body's PARENT-relative offset
    # stretches that segment while preserving the character A-pose direction.
    seg_scale: dict[str, float] = {bone: GLOBAL_HEIGHT_SCALE for bone, _, _ in CHAIN}

    def _seglen(a: str, b: str) -> float:
        return (_world_head(arm, a) - _world_head(arm, b)).length

    for side in ("L", "R"):
        up = f"CC_Base_{side}_Upperarm"
        fo = f"CC_Base_{side}_Forearm"
        ha = f"CC_Base_{side}_Hand"
        char_arm = _seglen(up, fo) + _seglen(fo, ha)   # shoulder->elbow->wrist
        if char_arm > 1e-6:
            s = PERFORMER_ARM_M / char_arm
            seg_scale[fo] = s   # offset Upperarm_head->Forearm_head == upper arm
            seg_scale[ha] = s   # offset Forearm_head ->Hand_head    == forearm
        th = f"CC_Base_{side}_Thigh"
        ca = f"CC_Base_{side}_Calf"
        char_thigh = _seglen(th, ca)
        if char_thigh > 1e-6:
            seg_scale[ca] = PERFORMER_THIGH_M / char_thigh   # offset Thigh->Calf == thigh

    identity = mathutils.Quaternion()  # (1,0,0,0)

    def emit_body(bone: str, parent: str | None) -> ET.Element:
        bname = name_of[bone]
        head_w = _world_head(arm, bone)
        Rb = rest_rot[bone]
        Rb_inv = Rb.inverted()
        Rpar = rest_rot[parent] if parent is not None else identity
        Rpar_inv = Rpar.inverted()

        # body pos is expressed in the PARENT body frame (= parent bone frame);
        # body quat is the child bone's rest rotation RELATIVE to the parent.
        # At rest (identity joints) this reproduces each bone's world frame, so
        # the joint qpos becomes exactly the bone-local displacement from rest.
        if parent is None:
            pos = head_w
        else:
            pos = Rpar_inv @ (head_w - _world_head(arm, parent))
            pos = pos * seg_scale[bone]   # P1: stretch this segment to performer size
        quat = Rpar_inv @ Rb
        body = ET.Element("body", name=bname, pos=_v(pos), quat=_q(quat))

        jt = joint_of[bone]
        if jt == "free":
            ET.SubElement(body, "freejoint", name=f"{bname}_free")
        elif jt == "ball":
            ET.SubElement(body, "joint", name=bname, type="ball", damping="1")
        elif jt == "hinge":
            ax_local = Rb_inv @ hinge_axis(bone)   # flexion axis in bone frame
            ET.SubElement(body, "joint", name=bname, type="hinge",
                          axis=_v(ax_local), damping="1", limited="false")
        elif jt == "weld":
            pass  # no joint: rigid offset from parent

        # capsule geom (inertia + visual). fromto in this body's (bone) frame.
        seg = Rb_inv @ (_world_tail(arm, bone) - head_w)
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


def _q(quat) -> str:
    # MuJoCo quaternion order is (w, x, y, z), same as mathutils.Quaternion.
    return f"{quat.w:.6f} {quat.x:.6f} {quat.y:.6f} {quat.z:.6f}"
