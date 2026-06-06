"""Live driver: ties tracker snapshot -> calibration -> mink solve -> bones.

This is the orchestration layer with NO Blender timer / modal logic, so it can
be unit-tested by injecting synthetic snapshots. The bpy modal operator (see
the add-on's operators) owns an instance of :class:`LiveDriver` and calls
:meth:`step` on each timer tick from the main thread.

Frames
------
``rig.build_mjcf`` builds the MuJoCo model in Blender WORLD meters, so a body's
rest origin (``body_xpos`` at qpos0) and a tracker snapshot (Blender-space via
``openvr_reader.svr_to_blender``) live in compatible meter frames. They differ
only by where the user stands vs. where the character is — a constant per-role
translation captured at A-pose by :class:`~openvr_reader.Calibration`. The IK
is position-only, so no rotation registration is attempted (deferred; see
docs/mink_pitfalls.md). v1 assumes the user faces the same way as the character.
"""
from __future__ import annotations

import mujoco
import numpy as np

try:  # normal: loaded as a package inside the Blender extension
    from . import apply as _apply
    from . import openvr_reader as _ovr
    from . import rig as _rig
    from . import solver as _solver
except ImportError:  # dev loop: src/*.py loaded standalone via importlib
    import sys
    _rig = sys.modules["manekko_rig"]
    _solver = sys.modules["manekko_solver"]
    _apply = sys.modules["manekko_apply"]
    _ovr = sys.modules["manekko_openvr"]


class LiveDriver:
    def __init__(self, arm, *, reader=None, solver_kwargs: dict | None = None):
        self.arm = arm
        self.rm = _rig.build_mjcf(arm)
        self.model = mujoco.MjModel.from_xml_string(self.rm.mjcf)
        self.solver = _solver.ManekkoSolver(
            self.model, self.rm.tracker_to_body,
            frame_types=self.rm.frame_types, **(solver_kwargs or {})
        )
        self.solver.reset_to_rest()
        self.solver.set_targets_from_current()
        self.reader = reader if reader is not None else _ovr.TrackerReader()
        self._fk = self._build_fk_tables()
        self.calibration: _ovr.Calibration | None = None
        self.last_q: np.ndarray | None = None
        self.last_error: str | None = None
        # finger bones per hand (driven by the controller trigger, not the IK)
        self.finger_names = _apply.finger_bone_names(arm)

    # -- world-FK (forearm from the controller, post-IK) ----------------
    def _build_fk_tables(self) -> dict:
        """Precompute per-FK-role: parent (upperarm) body id, forearm ball joint
        qpos address, and the constant rest rotation of the forearm relative to
        the upperarm (rel_rest = R_up_rest^T @ R_fore_rest, 3x3). See
        rig.FK_ORIENT_BONE / _apply_forearm_fk."""
        out = {}
        for role, (fore_body, up_body) in self.rm.fk_orient.items():
            up_bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, up_body)
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, fore_body)
            if up_bid < 0 or jid < 0:
                continue
            adr = int(self.model.jnt_qposadr[jid])
            fore_bone = self.rm.body_to_bone[fore_body]
            up_bone = self.rm.body_to_bone[up_body]
            R_up_rest = np.array(self.rm.rest_world[up_bone].to_quaternion()
                                 .to_matrix())
            R_fore_rest = np.array(self.rm.rest_world[fore_bone].to_quaternion()
                                   .to_matrix())
            rel_rest = R_up_rest.T @ R_fore_rest
            out[role] = (up_bid, adr, rel_rest)
        return out

    def _apply_forearm_fk(self, orientations: dict[str, np.ndarray]) -> None:
        """Set each forearm ball joint so the forearm reaches the controller's
        WORLD orientation exactly, using the ACTUAL post-solve parent (upperarm)
        world orientation (so mink/torso error is absorbed; no accumulation).
        q_fore = rel_rest^T @ R_up_actual^T @ R_fore_target."""
        cfg = self.solver.configuration
        data = cfg.data
        q = cfg.q.copy()
        changed = False
        for role, (up_bid, adr, rel_rest) in self._fk.items():
            R_target = orientations.get(role)
            if R_target is None:
                continue
            R_up = np.array(data.xmat[up_bid]).reshape(3, 3)
            Rq = rel_rest.T @ R_up.T @ np.asarray(R_target, float)
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, np.ascontiguousarray(Rq).reshape(9))
            q[adr:adr + 4] = quat
            changed = True
        if changed:
            cfg.update(q)

    # -- rest geometry --------------------------------------------------
    def body_rest_positions(self) -> dict[str, np.ndarray]:
        """Each tracker role's body origin at the rest (A) pose, world meters."""
        self.solver.reset_to_rest()
        data = self.solver.configuration.data
        out = {}
        for role, body in self.rm.tracker_to_body.items():
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
            out[role] = np.array(data.xpos[bid])
        return out

    def body_rest_orientations(self) -> dict[str, np.ndarray]:
        """Each tracker role's body world orientation (3x3) at the rest (A) pose."""
        self.solver.reset_to_rest()
        data = self.solver.configuration.data
        out = {}
        for role, body in self.rm.tracker_to_body.items():
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
            out[role] = np.array(data.xmat[bid]).reshape(3, 3)
        # world-FK roles (controllers): rest orientation = the FOREARM rest, so
        # Calibration registers the controller mount offset against it.
        for role, (fore_body, _up_body) in self.rm.fk_orient.items():
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, fore_body)
            out[role] = np.array(data.xmat[bid]).reshape(3, 3)
        return out

    # -- calibration ----------------------------------------------------
    def calibrate(self, snapshot: dict[str, np.ndarray] | None = None,
                  snapshot_rot: dict[str, np.ndarray] | None = None) -> _ovr.Calibration:
        """Record A-pose offsets from the current tracker snapshot.

        Only roles present in the snapshot are calibrated; absent roles get no
        offset and their targets are ignored until calibrated. If orientations
        (``snapshot_rot``) are supplied, an orientation mount offset is also
        registered for each role (used only by orientation-tracked roles).
        """
        if snapshot is None:
            snapshot = self.reader.snapshot()
        rest = self.body_rest_positions()
        rest_rot = self.body_rest_orientations()
        self.calibration = _ovr.Calibration.from_apose(
            snapshot, rest, snapshot_rot, rest_rot)
        return self.calibration

    # -- per-frame step -------------------------------------------------
    def step(self, snapshot: dict[str, np.ndarray] | None = None,
             snapshot_rot: dict[str, np.ndarray] | None = None,
             grip: dict[str, float] | None = None,
             *, dt: float = 1.0 / 60.0, iters: int = 4):
        """One solve+apply tick. Returns True on success, False if held.

        Keeps the previous pose on an empty snapshot or a solver failure
        (mink raises on infeasible QPs) so the live view never snaps to rest.
        """
        if snapshot is None:
            snapshot = self.reader.snapshot()
        if not snapshot:
            return False

        if self.calibration is not None:
            targets = self.calibration.apply(snapshot)
            orientations = self.calibration.apply_rot(snapshot_rot) if snapshot_rot else {}
        else:
            targets = snapshot  # uncalibrated: drive raw (mostly for testing)
            orientations = {}

        self.solver.set_target_poses(targets, orientations)
        try:
            self.last_q = self.solver.solve(dt=dt, iters=iters)
            self.last_error = None
        except Exception as e:  # mink NoSolutionFound etc. -> hold last pose
            self.last_error = repr(e)
            return False

        # Post-IK world-FK: set the forearms directly from the controller world
        # orientation (reads the actual solved upperarm; no second mink pass).
        if orientations:
            self._apply_forearm_fk(orientations)

        _apply.apply_pose(self.arm, self.rm, self.solver.configuration,
                          fingers=grip, finger_names=self.finger_names)
        return True

    def apply_rest(self):
        """Reset the solver to the rest (A) pose and apply it to the bones."""
        self.solver.reset_to_rest()
        _apply.apply_pose(self.arm, self.rm, self.solver.configuration)

    # -- lifecycle ------------------------------------------------------
    def start_reader(self):
        self.reader.start()

    def stop_reader(self):
        self.reader.stop()
