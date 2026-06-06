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
    # Forearm = BALL (v0.4): a 3-DOF joint, but NOT a mink target. mink/posture
    # parks it near rest; AFTER the solve we OVERWRITE its qpos directly from the
    # controller's world orientation (world-FK; see FK_ORIENT_BONE + live._apply_
    # forearm_fk). Being downstream + untargeted, it provably does not affect the
    # upperarm solve. Hand = WELD (the wrist is held rigid this version; the 2-axis
    # trackpad wrist comes later -> Hand becomes a driven joint then).
    ("CC_Base_L_Forearm",   "CC_Base_L_Upperarm",  "ball"),
    ("CC_Base_L_Hand",      "CC_Base_L_Forearm",   "weld"),
    ("CC_Base_R_Clavicle",  "CC_Base_Spine02",     "weld"),
    ("CC_Base_R_Upperarm",  "CC_Base_R_Clavicle",  "ball"),
    ("CC_Base_R_Forearm",   "CC_Base_R_Upperarm",  "ball"),   # BALL, world-FK (see L_Forearm)
    ("CC_Base_R_Hand",      "CC_Base_R_Forearm",   "weld"),
]

# tracker role -> bone whose body frame the FrameTask targets
TRACKER_TO_BONE: dict[str, str] = {
    "hip":     "CC_Base_Hip",
    "head":    "CC_Base_Head",
    "foot_l":  "CC_Base_L_Foot",
    "foot_r":  "CC_Base_R_Foot",
    # 2026-06-07: the WRIST trackers (hand_l/r) are REMOVED from the IK this
    # version — still read by openvr_reader, but no FrameTask. They return later
    # via a MuJoCo-FK post-correction (separate instructions). The CHEST target
    # is also dropped: that tracker is now an ELBOW (see ELBOW_SITE_BONE). The
    # chest bone (Spine02) is still solved implicitly from the 4 torso/arm
    # targets (hip, head, elbow_l, elbow_r) via the welded clavicles.
    #   "hand_l":  "CC_Base_L_Hand",   # wrist tracker — deferred
    #   "hand_r":  "CC_Base_R_Hand",
    #   "chest":   "CC_Base_Spine02",  # repurposed to elbow_l
    # knee trackers stay DROPPED (legs driven by the foot targets; Posture
    # resolves the knee swivel toward the A-pose). knee_r's tracker is now an
    # elbow (see ELBOW_ORIENT_BONE).
    #   "knee_l":  "CC_Base_L_Calf",
}

# --- Elbow trackers (2026-06-07, step 2): ORIENTATION-only on the upperarm -----
# The elbow tracker is strapped rigidly on the UPPERARM, so it measures the
# upperarm's WORLD ORIENTATION directly and exactly (all 3 DOF, incl. the TWIST
# that makes "the elbow point up"). The upperarm is a 3-DOF shoulder ball, so we
# drive it with an ORIENTATION-ONLY FrameTask on the Upperarm BODY (NO position
# target). Position-only-on-a-site (v0.3.1) constrained only the direction and
# left the twist free; adding a soft orientation on top (v0.3.2) fought the
# position target and gave a compromise (~17deg of 40). Orientation-only on the
# 3-DOF joint has no competing constraint -> the upperarm matches the tracker
# EXACTLY (measured: 0.01deg error at cost 5.0). The shoulder POSITION comes from
# the torso solve (hip + head); the welded forearm follows at rest. The elbow
# location is then shoulder + exact-upperarm-orientation * length, which is
# correct. solver puts these roles in `orientation_only_roles`.
ELBOW_ORIENT_BONE: dict[str, str] = {
    "elbow_l": "CC_Base_L_Upperarm",
    "elbow_r": "CC_Base_R_Upperarm",
}

# --- Forearm world-FK from the CONTROLLER (v0.4) -------------------------------
# The hand controller is held rigidly, so it measures the forearm's WORLD
# orientation. We do NOT add it to the mink IK; instead, AFTER the solve we set
# the forearm ball joint directly so the forearm reaches the controller's world
# orientation EXACTLY. Crucially the local joint angle is computed from the
# ACTUAL post-solve parent (upperarm) world orientation, so any mink/torso
# imperfection is absorbed (self-correcting) and there is no cumulative error
# (absolute per frame, never integrated). role -> (forearm bone, parent upperarm
# bone). The controller orientation source is the palm role (already read).
# The 9 cm controller-past-the-wrist offset is irrelevant here: orientation is
# origin-independent. The wrist tracker stays free for future use; if the
# controller proves unreliable, swap the source role back to hand_l/r.
FK_ORIENT_BONE: dict[str, tuple[str, str]] = {
    "palm_l": ("CC_Base_L_Forearm", "CC_Base_L_Upperarm"),
    "palm_r": ("CC_Base_R_Forearm", "CC_Base_R_Upperarm"),
}

# hinge axes (elbow/knee) are derived from the rest geometry per-bone below.


# --- P2: performer-MEASURED model (build the PERFORMER's FK from real joint
#     coordinates, not the character's proportions). 2026-06-06 rewrite. ---
# Earlier (P1) we stretched the whole character uniformly by a single height
# ratio (1.80/1.59) and overrode only the arm & thigh LENGTHS. That left every
# joint CENTER (shoulder/hip placement, torso heights, shoulder/hip widths) on
# the character's proportions -- the deferred "fulcrum" problem -- which made the
# wrist fall short when the arm was raised (the model shoulder sat at the wrong
# height/width). The performer is NOT proportioned like the character, so we now
# place each joint at the performer's MEASURED world coordinate.
#
# Method: reconstruct each bone's HEAD position in armature/world space
# (Z = floor height, X = lateral half-width, Y = depth) from the measurements
# below, then build the body offsets from those. We keep from the character only:
#   * each bone's rest ORIENTATION (rest_rot) -- the A-pose direction proxy, so
#     apply.py is unchanged and the solved qpos stays the performer's joint angle;
#   * the DEPTH (Y) of each joint and the intermediate spine/neck/head joint
#     heights, which are unmeasured (the spinal curve shape). Position calibration
#     absorbs the constant tracker<->joint offset, so this proxy is harmless.
# Limbs (arm/leg) are walked along the character A-pose direction with the
# performer's measured segment LENGTH; the shoulder & hip JOINTS are placed at the
# measured height + half-width directly (the 2D placement a scalar scale can't do).
#
# All meters, performer, measured 2026-06-06 (stature 1.80 m). Set these to the
# character's own joint coordinates to fall back to the pure character model.
PERFORMER = {
    "hip_height":          0.90,   # floor -> hip joint (greater trochanter)
    "chest_height":        1.27,   # floor -> Spine02 origin (sternum tracker level)
    "shoulder_height":     1.45,   # floor -> shoulder joint (acromion)
    "hip_half_width":      0.135,  # body center -> hip joint, lateral (0.27 / 2)
    "shoulder_half_width": 0.20,   # body center -> shoulder joint, lateral (0.40 / 2)
    "upperarm":            0.30,   # shoulder -> elbow joint (real bone, 2026-06-07).
                                   # The elbow tracker sits at ELBOW_SITE_M=0.24
                                   # along this; the elbow joint itself is FK.
    # forearm: NOT a target this version (welded, held at rest). Value is only
    # cosmetic placement of the welded forearm/hand. Restore the tracker-lever
    # reasoning here when the wrist tracker returns.
    "forearm":             0.26,
    "thigh":               0.47,   # hip joint -> knee
    "shin":                0.46,   # knee -> ankle
}


def _performer_heads(arm) -> dict[str, "mathutils.Vector"]:
    """Reconstruct each chain bone's HEAD position (armature/world meters) from
    the performer's measured joint coordinates. See the PERFORMER notes above for
    what is measured vs. kept from the character (depth, orientation, intermediate
    spine joint heights)."""
    P = PERFORMER
    ch = {bone: _world_head(arm, bone) for bone, _, _ in CHAIN}
    perf: dict[str, mathutils.Vector] = {}

    def V(x, y, z):
        return mathutils.Vector((x, y, z))

    # --- torso: anchor hip & chest at measured heights; keep character X (center)
    #     and Y (depth). Intermediate spine joints interpolate by char fraction. ---
    z_hip_c, z_chest_c = ch["CC_Base_Hip"].z, ch["CC_Base_Spine02"].z
    hipZ, chestZ = P["hip_height"], P["chest_height"]
    rate = (chestZ - hipZ) / (z_chest_c - z_hip_c) if abs(z_chest_c - z_hip_c) > 1e-9 else 1.0

    perf["CC_Base_Hip"] = V(ch["CC_Base_Hip"].x, ch["CC_Base_Hip"].y, hipZ)
    for b in ("CC_Base_Waist", "CC_Base_Spine01"):
        f = (ch[b].z - z_hip_c) / (z_chest_c - z_hip_c) if abs(z_chest_c - z_hip_c) > 1e-9 else 0.0
        perf[b] = V(ch[b].x, ch[b].y, hipZ + f * (chestZ - hipZ))
    perf["CC_Base_Spine02"] = V(ch["CC_Base_Spine02"].x, ch["CC_Base_Spine02"].y, chestZ)
    # neck/head joints: unmeasured (the head tracker height is a surface point, not
    # a joint) -> keep character height above the chest, scaled by the torso rate.
    for b in ("CC_Base_NeckTwist01", "CC_Base_Head"):
        perf[b] = V(ch[b].x, ch[b].y, chestZ + (ch[b].z - z_chest_c) * rate)
    # pelvis weld: keep the character offset from the hip (scaled by the torso rate)
    pb = "CC_Base_Pelvis"
    perf[pb] = V(ch[pb].x, ch[pb].y, hipZ + (ch[pb].z - z_hip_c) * rate)

    # --- shoulders + arms ---
    for side in ("L", "R"):
        sp, cl = "CC_Base_Spine02", f"CC_Base_{side}_Clavicle"
        up, fo, ha = (f"CC_Base_{side}_Upperarm", f"CC_Base_{side}_Forearm",
                      f"CC_Base_{side}_Hand")
        sx = 1.0 if ch[up].x >= 0 else -1.0
        shoulder = V(sx * P["shoulder_half_width"], ch[up].y, P["shoulder_height"])
        perf[up] = shoulder
        # clavicle (weld) sits between the chest and the shoulder by char fraction
        denom = (ch[up] - ch[sp]).length
        t = (ch[cl] - ch[sp]).length / denom if denom > 1e-9 else 0.5
        perf[cl] = perf[sp] + (shoulder - perf[sp]) * t
        # elbow & wrist: walk the character arm direction with performer lengths
        elbow = shoulder + (ch[fo] - ch[up]).normalized() * P["upperarm"]
        perf[fo] = elbow
        perf[ha] = elbow + (ch[ha] - ch[fo]).normalized() * P["forearm"]

    # --- hips + legs ---
    for side in ("L", "R"):
        th, ca, ft = (f"CC_Base_{side}_Thigh", f"CC_Base_{side}_Calf",
                      f"CC_Base_{side}_Foot")
        sx = 1.0 if ch[th].x >= 0 else -1.0
        hip_joint = V(sx * P["hip_half_width"], ch[th].y, P["hip_height"])
        perf[th] = hip_joint
        knee = hip_joint + (ch[ca] - ch[th]).normalized() * P["thigh"]
        perf[ca] = knee
        perf[ft] = knee + (ch[ft] - ch[ca]).normalized() * P["shin"]

    return perf


@dataclass
class RigModel:
    mjcf: str
    body_to_bone: dict[str, str]          # mjcf body name -> blender bone name
    bone_to_body: dict[str, str]
    tracker_to_body: dict[str, str]       # role -> mjcf frame name (body or site)
    rest_world: dict[str, mathutils.Matrix]  # bone -> armature/world rest matrix
    bodies: list[str] = field(default_factory=list)
    frame_types: dict[str, str] = field(default_factory=dict)  # role -> "body"|"site"
    # world-FK roles: role -> (forearm mjcf body, parent upperarm mjcf body)
    fk_orient: dict[str, tuple[str, str]] = field(default_factory=dict)


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

    # P2: performer-measured joint coordinates (replaces the global height stretch
    # + per-limb scalar scales). Each body offset is built from these heads below.
    perf_head = _performer_heads(arm)

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
            pos = perf_head[bone]   # root: absolute performer hip position
        else:
            # P2: offset from the performer-measured heads, expressed in the parent
            # bone frame (rest_rot kept from the character, so apply.py is unchanged).
            pos = Rpar_inv @ (perf_head[bone] - perf_head[parent])
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

    tracker_to_body = {role: name_of[bone] for role, bone in TRACKER_TO_BONE.items()}
    # elbow roles target the Upperarm BODY (orientation-only; see solver
    # orientation_only_roles). All frames are bodies now.
    for role, bone in ELBOW_ORIENT_BONE.items():
        tracker_to_body[role] = name_of[bone]
    frame_types = {role: "body" for role in tracker_to_body}

    fk_orient = {role: (name_of[fore], name_of[up])
                 for role, (fore, up) in FK_ORIENT_BONE.items()}

    return RigModel(
        mjcf=mjcf,
        body_to_bone=body_to_bone,
        bone_to_body={v: k for k, v in body_to_bone.items()},
        tracker_to_body=tracker_to_body,
        rest_world=rest_world,
        bodies=[name_of[b] for b, _, _ in CHAIN],
        frame_types=frame_types,
        fk_orient=fk_orient,
    )


def _v(vec) -> str:
    return f"{vec[0]:.6f} {vec[1]:.6f} {vec[2]:.6f}"


def _q(quat) -> str:
    # MuJoCo quaternion order is (w, x, y, z), same as mathutils.Quaternion.
    return f"{quat.w:.6f} {quat.x:.6f} {quat.y:.6f} {quat.z:.6f}"
