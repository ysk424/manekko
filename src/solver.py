"""mink full-body IK wrapper for the reduced CC_Base MuJoCo model.

10 tracker roles drive position-only FrameTasks; a low-cost PostureTask
regularizes toward the A-pose rest configuration. Solved as a QP with daqp.
"""
from __future__ import annotations

import mink
import mujoco
import numpy as np


class ManekkoSolver:
    def __init__(
        self,
        model: "mujoco.MjModel",
        tracker_to_body: dict[str, str],
        *,
        position_cost: float = 1.0,
        # Kept as a separate knob for the hands (VIVE controllers). In v0.0.4 we
        # down-weighted them to 0.2 to stop the imperfect controller wrist target
        # from pinning the elbow tracker. In v0.0.5 the elbow trackers were
        # DROPPED from the IK (rig.TRACKER_TO_BONE), so the over-constraint is
        # gone: the wrist now drives the whole arm and the PostureTask resolves
        # the elbow swivel. So the hands go back to full weight for tight wrist
        # tracking. Pure scalar weight, no angles.
        hand_position_cost: float = 1.0,
        # Roles that additionally track ORIENTATION, not just position. Staged
        # rollout of rotation (CLAUDE.md "角度は最後に・段階的に"): head + hip
        # first — single rigid pieces (the pelvis root sets the body facing, the
        # head its gaze), no twist coupling. Expand to feet/limbs once verified.
        # The orientation target comes from the tracker pose, registered at the
        # A-pose (Calibration.rot_offset); see live.LiveDriver.step.
        orientation_roles: tuple[str, ...] = ("head", "hip"),
        # Kept low vs position_cost (1.0): a gentle orientation pull so a slightly
        # off registration can't whip the body around (the owner's "ぐるぐる回る"
        # risk). Raise once head+hip prove stable.
        orientation_cost: float = 1e-1,
        # 1e-1 (not 1e-2): position-only differential IK leaves the body's
        # twist/redundant DOFs unconstrained, so velocity integration drifts
        # (path-dependent null-space wind-up — cyclic motion like marching
        # ratchets the spine/arms into a twist; reversible by moving the other
        # way). The PostureTask regularizes toward the rest A-pose each frame,
        # giving the null space an absolute anchor. 1e-1 killed the observed
        # drift live (2026-05-30) with acceptable tracking. See
        # docs/mink_pitfalls.md "null-space drift".
        posture_cost: float = 1e-1,
        solver: str = "daqp",
        damping: float = 1e-1,
    ) -> None:
        self.model = model
        self.solver = solver
        self.damping = damping
        self.configuration = mink.Configuration(model)
        self.configuration.update(model.qpos0)

        hand_roles = ("hand_l", "hand_r")
        self.frame_tasks: dict[str, mink.FrameTask] = {}
        for role, body in tracker_to_body.items():
            pc = hand_position_cost if role in hand_roles else position_cost
            oc = orientation_cost if role in orientation_roles else 0.0
            t = mink.FrameTask(
                frame_name=body,
                frame_type="body",
                position_cost=pc,
                orientation_cost=oc,    # 0 = position-only (default for most roles)
                lm_damping=1.0,
            )
            self.frame_tasks[role] = t

        self.posture_task = mink.PostureTask(model, cost=posture_cost)
        self.posture_task.set_target(model.qpos0)

        self.tasks = list(self.frame_tasks.values()) + [self.posture_task]

    def reset_to_rest(self) -> None:
        self.configuration.update(self.model.qpos0)

    def set_target_poses(
        self,
        positions: dict[str, np.ndarray],
        orientations: dict[str, np.ndarray] | None = None,
    ) -> None:
        """positions: role -> world xyz (m). orientations: role -> 3x3 world
        rotation (optional). Sets each FrameTask target SE3; a role with no
        orientation gets identity rotation (ignored anyway when its
        orientation_cost is 0). Missing roles keep their prior target."""
        orientations = orientations or {}
        for role, pos in positions.items():
            task = self.frame_tasks.get(role)
            if task is None:
                continue
            R = orientations.get(role)
            rot = mink.SO3.from_matrix(np.asarray(R, float)) if R is not None \
                else mink.SO3.identity()
            task.set_target(mink.SE3.from_rotation_and_translation(
                rot, np.asarray(pos, dtype=float)))

    def set_target_positions(self, positions: dict[str, np.ndarray]) -> None:
        """Position-only convenience: identity orientation targets."""
        self.set_target_poses(positions)

    def set_targets_from_current(self) -> None:
        """Initialize every FrameTask target to the current body pose (rest)."""
        for task in self.frame_tasks.values():
            task.set_target_from_configuration(self.configuration)

    def solve(self, dt: float = 1.0 / 60.0, iters: int = 4) -> np.ndarray:
        for _ in range(iters):
            vel = mink.solve_ik(
                self.configuration, self.tasks, dt, self.solver, self.damping
            )
            self.configuration.integrate_inplace(vel, dt)
        return self.configuration.q

    def body_xpos(self, body: str) -> np.ndarray:
        data = self.configuration.data
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
        return np.array(data.xpos[bid])
