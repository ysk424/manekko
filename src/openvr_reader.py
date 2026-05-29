"""SteamVR / VIVE Tracker 3.0 pose source for manekko (no bpy here).

A background thread polls OpenVR for device poses and stores the latest
snapshot. The Blender main thread reads :meth:`TrackerReader.snapshot` each
frame (cheap, lock-guarded) and never touches OpenVR directly — keeping all
``bpy`` work on the main thread and all ``openvr`` work off it.

Coordinate frames
-----------------
SteamVR is right-handed **Y-up**, meters: +X right, +Y up, +Z toward the
viewer (i.e. "backward"). Blender is right-handed **Z-up**, meters: +X right,
+Y forward, +Z up. Both are meters, so scale is 1 and we only swap axes::

    (x, y, z)_steamvr  ->  (x, -z, y)_blender

Registration to the character (where the user stands vs. where the CC body
is) is absorbed by :class:`Calibration`, recorded once while the user holds
the A-pose. Because the IK is position-only, no rotation offset is needed.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

import numpy as np

# The 10 tracker roles, in the order rig.TRACKER_TO_BONE expects.
ROLES: tuple[str, ...] = (
    "hip", "head",
    "hand_l", "hand_r",
    "foot_l", "foot_r",
    "elbow_l", "elbow_r",
    "knee_l", "knee_r",
)

# Known VIVE Tracker serial -> role assignments. Partially known; the rest are
# assigned at runtime via TrackerReader.assign() / the N-panel. Serials are the
# Prop_SerialNumber_String values reported by OpenVR.
SERIAL_TO_ROLE: dict[str, str] = {
    # hands are the two VIVE controllers (they also carry the trigger button
    # used to define the A-rest pose / start-stop recording); the other 8 are
    # VIVE Tracker 3.0. Confirmed by floor line-up 2026-05-30.
    "LHR-9EFF8645": "hand_r",   # R controller
    "LHR-0B253252": "hand_l",   # L controller
    "LHR-CC5F5D2C": "head",
    "LHR-15E5788A": "hip",      # drives the root bone's global position
    "LHR-4CEBC3D1": "elbow_r",
    "LHR-8CBC92B3": "elbow_l",
    "LHR-31597DDE": "knee_r",
    "LHR-4BDF9009": "knee_l",
    "LHR-60481EF9": "foot_r",
    "LHR-9E4926DA": "foot_l",
}

# The controller whose trigger would define the A-rest pose / toggle recording.
# NOTE (2026-05-30): the legacy IVRSystem.getControllerState returns NO live
# button data here (packet number never advances) even though the controller is
# awake — modern SteamVR routes input through the action-based IVRInput API. So
# the trigger is NOT usable via the legacy path; switching to IVRInput is
# deferred. Current recording scheme uses BEEPS instead of the trigger:
#   start -> beep1 -> 5 s (performer takes A-pose) -> beep2 (calibrate + record).
TRIGGER_SERIALS: tuple[str, ...] = ("LHR-9EFF8645", "LHR-0B253252")


def svr_to_blender(p) -> np.ndarray:
    """SteamVR (Y-up, m) position -> Blender (Z-up, m). Scale 1, pure axis swap."""
    x, y, z = float(p[0]), float(p[1]), float(p[2])
    return np.array((x, -z, y), dtype=float)


# Front alignment: the capture room has the computer screen at world -X and the
# performer faces the screen, so "forward" = world -X must map to the character
# front (Blender -Y). svr_to_blender alone maps world -X -> Blender -X (90 deg
# off), so a fixed +90 deg yaw about Z is applied afterwards. Net combined map:
# SteamVR (x, y, z) -> Blender (z, x, y). No mirror (verified). Live-verified
# 2026-05-30: forward->-Y, performer-left->+X both correct.
FRONT_YAW_DEG = 90.0


def world_to_blender(p) -> np.ndarray:
    """SteamVR world position -> front-aligned Blender position (screen -X = front -Y).

    Equivalent to svr_to_blender() followed by a +90 deg yaw about Z. Apply this
    consistently to both the calibration reference and live positions (the delta
    is what drives the character). Assumes the performer faces the screen (-X);
    if the room layout changes, derive the yaw from the hip tracker heading.
    """
    x, y, z = float(p[0]), float(p[1]), float(p[2])
    return np.array((z, x, y), dtype=float)


def _mat34_pos(m) -> tuple[float, float, float]:
    """Translation column of an OpenVR HmdMatrix34_t (row-major 3x4)."""
    return (m[0][3], m[1][3], m[2][3])


@dataclass
class Calibration:
    """Per-role position offset added to raw (Blender-space) tracker positions.

    Recorded while the user stands in the A-pose: for each role we store
    ``offset = body_rest_pos - raw_tracker_pos`` so that, at calibration time,
    the resulting IK target lands exactly on the body's rest origin. Live
    targets then follow the tracker rigidly: ``target = raw + offset``.
    """

    offset: dict[str, np.ndarray] = field(default_factory=dict)

    @classmethod
    def from_apose(
        cls,
        raw_positions: dict[str, np.ndarray],
        body_rest_positions: dict[str, np.ndarray],
    ) -> "Calibration":
        off = {}
        for role, raw in raw_positions.items():
            rest = body_rest_positions.get(role)
            if rest is not None:
                off[role] = np.asarray(rest, float) - np.asarray(raw, float)
        return cls(offset=off)

    def apply(self, raw_positions: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        out = {}
        for role, raw in raw_positions.items():
            o = self.offset.get(role)
            out[role] = np.asarray(raw, float) + (o if o is not None else 0.0)
        return out


@dataclass
class DeviceInfo:
    idx: int
    cls: str          # "HMD" | "Controller" | "GenericTracker" | "TrackingReference"
    serial: str
    model: str
    connected: bool
    pose_valid: bool
    role: str | None  # assigned tracker role, if any


class TrackerReader:
    """Background OpenVR pose poller.

    Usage (main thread)::

        r = TrackerReader()
        r.start()
        ...
        snap = r.snapshot()          # {role: np.array([x,y,z])} in Blender meters
        ...
        r.stop()

    Nothing here imports ``bpy``; safe to run off the main thread.
    """

    def __init__(
        self,
        serial_to_role: dict[str, str] | None = None,
        *,
        rate_hz: float = 90.0,
        predicted_dt: float = 0.0,
    ) -> None:
        self._serial_to_role = dict(SERIAL_TO_ROLE)
        if serial_to_role:
            self._serial_to_role.update(serial_to_role)
        self._period = 1.0 / max(rate_hz, 1.0)
        self._predicted_dt = predicted_dt

        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

        # shared state (guarded by _lock)
        self._latest: dict[str, np.ndarray] = {}   # role -> raw Blender-space pos
        self._devices: list[DeviceInfo] = []
        self._status: str = "stopped"               # stopped|running|error
        self._error: str | None = None
        self._frame: int = 0                         # increments each poll

    # -- lifecycle -------------------------------------------------------
    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._loop, name="manekko-openvr", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        t = self._thread
        if t:
            t.join(timeout)
        self._thread = None

    # -- main-thread reads ----------------------------------------------
    def snapshot(self) -> dict[str, np.ndarray]:
        """Latest raw (uncalibrated) Blender-space positions per assigned role."""
        with self._lock:
            return {k: v.copy() for k, v in self._latest.items()}

    def device_table(self) -> list[DeviceInfo]:
        with self._lock:
            return list(self._devices)

    @property
    def status(self) -> str:
        with self._lock:
            return self._status

    @property
    def error(self) -> str | None:
        with self._lock:
            return self._error

    def assign(self, serial: str, role: str | None) -> None:
        """Bind a tracker serial to a role (or pass role=None to unassign)."""
        with self._lock:
            if role is None:
                self._serial_to_role.pop(serial, None)
            else:
                self._serial_to_role[serial] = role

    # -- worker ----------------------------------------------------------
    def _loop(self) -> None:
        import openvr

        cls_names = {
            openvr.TrackedDeviceClass_Invalid: "Invalid",
            openvr.TrackedDeviceClass_HMD: "HMD",
            openvr.TrackedDeviceClass_Controller: "Controller",
            openvr.TrackedDeviceClass_GenericTracker: "GenericTracker",
            openvr.TrackedDeviceClass_TrackingReference: "TrackingReference",
            getattr(openvr, "TrackedDeviceClass_DisplayRedirect", 5): "DisplayRedirect",
        }
        vr = None
        try:
            vr = openvr.init(openvr.VRApplication_Background)
            with self._lock:
                self._status = "running"
                self._error = None

            n = openvr.k_unMaxTrackedDeviceCount
            serial_cache: dict[int, tuple[str, str, str]] = {}  # idx -> (serial, model, cls)

            def device_meta(i: int) -> tuple[str, str, str]:
                if i in serial_cache:
                    return serial_cache[i]
                try:
                    serial = vr.getStringTrackedDeviceProperty(
                        i, openvr.Prop_SerialNumber_String)
                    model = vr.getStringTrackedDeviceProperty(
                        i, openvr.Prop_ModelNumber_String)
                except Exception:
                    serial, model = "", ""
                cls = cls_names.get(vr.getTrackedDeviceClass(i), "Unknown")
                meta = (serial, model, cls)
                if serial:
                    serial_cache[i] = meta
                return meta

            while not self._stop.is_set():
                t0 = time.perf_counter()
                poses = vr.getDeviceToAbsoluteTrackingPose(
                    openvr.TrackingUniverseStanding, self._predicted_dt, n)

                with self._lock:
                    role_map = dict(self._serial_to_role)

                latest: dict[str, np.ndarray] = {}
                devices: list[DeviceInfo] = []
                for i in range(n):
                    p = poses[i]
                    connected = bool(p.bDeviceIsConnected)
                    if not connected:
                        serial_cache.pop(i, None)
                        continue
                    serial, model, cls = device_meta(i)
                    role = role_map.get(serial)
                    valid = bool(p.bPoseIsValid)
                    devices.append(DeviceInfo(
                        idx=i, cls=cls, serial=serial, model=model,
                        connected=connected, pose_valid=valid, role=role,
                    ))
                    if role and valid:
                        pos = _mat34_pos(p.mDeviceToAbsoluteTracking)
                        latest[role] = svr_to_blender(pos)

                with self._lock:
                    self._latest = latest
                    self._devices = devices
                    self._frame += 1

                dt = time.perf_counter() - t0
                if dt < self._period:
                    self._stop.wait(self._period - dt)
        except Exception as e:  # noqa: BLE001 - report, don't crash Blender
            with self._lock:
                self._status = "error"
                self._error = repr(e)
        finally:
            if vr is not None:
                try:
                    openvr.shutdown()
                except Exception:
                    pass
            with self._lock:
                if self._status != "error":
                    self._status = "stopped"
