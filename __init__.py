"""Manekko — VIVE Tracker full-body mocap → CC character via mink IK.

Blender 5.1 extension. The bundled wheels (mink/mujoco/openvr/...) are supposed
to be installed and put on the import path by Blender's own extension wheel
system (manifest ``wheels = [...]``). In practice that does NOT work in this
environment — with the manifest wheels alone, ``import openvr`` raises
ModuleNotFoundError (verified 2026-05-31 on v0.1.3, both before and after the
NVIDIA-era note). So we self-bootstrap: ``_ensure_wheels()`` extracts the bundled
``./wheels/*.whl`` once into ``./_libs`` and prepends it to ``sys.path`` (no-op
if the deps already import). numpy is never bundled, so it resolves to Blender's.

This trips Blender 5.1's "no sys.path modification" policy, shown as a warning
("Policy violation with sys.path: ._libs"). That warning is COSMETIC: it does
not block the extension and it is NOT why the Add-ons UI lacks an uninstall
button (removing the sys.path use in v0.1.3 cleared the warning but the uninstall
button was still absent — unrelated). Uninstall by deleting the install folder.

N-panel "Manekko" (3D View sidebar): Start/Stop, Record, Calibrate. See
src/ops.py.
"""
from __future__ import annotations

import glob
import os
import sys
import zipfile

import bpy
from bpy.props import BoolProperty, EnumProperty, IntProperty, StringProperty
from bpy.types import WindowManager


# The bundled deps that must resolve to ./_libs (never a stray dev copy).
_BUNDLED_DEPS = ("mujoco", "mink", "openvr")


def _ensure_wheels() -> None:
    """Guarantee the bundled deps are importable, independent of Blender's wheel
    system (which does not surface them here). Extracts ./wheels/*.whl into
    ./_libs once (excluding numpy, never bundled) and forces ./_libs to the FRONT
    of sys.path so the bundled packages always win.

    Hardened (v0.3.4) against DEV-PATH SHADOWS: the dev-loop / MCP
    ``Temp\\manekko_libs`` (CLAUDE.md) may be on sys.path with a PARTIAL copy of a
    dep — a ``mujoco`` dir without ``__init__.py`` (a namespace package: no
    ``__file__`` / no ``MjModel``), or an ``openvr`` missing ``version.py``
    (``ModuleNotFoundError: No module named 'openvr.version'``). Either shadows
    the good ``_libs`` copy. So we (1) force ``_libs`` first, (2) purge any cached
    dep NOT loaded from ``_libs`` so it re-imports cleanly, and (3) invalidate the
    import finder caches after the sys.path change."""
    here = os.path.dirname(__file__)
    wheels_dir = os.path.join(here, "wheels")
    libs = os.path.join(here, "_libs")

    if os.path.isdir(wheels_dir):
        marker = os.path.join(libs, ".extracted")
        if not os.path.exists(marker):
            os.makedirs(libs, exist_ok=True)
            for whl in sorted(glob.glob(os.path.join(wheels_dir, "*.whl"))):
                try:
                    with zipfile.ZipFile(whl) as z:
                        z.extractall(libs)
                except Exception as e:  # noqa: BLE001
                    print(f"[manekko] failed to extract {os.path.basename(whl)}: {e!r}")
            with open(marker, "w") as f:
                f.write("ok\n")

    # _libs must be FIRST so the bundled packages win over any stray dev path.
    if libs in sys.path:
        sys.path.remove(libs)
    sys.path.insert(0, libs)

    # Purge any cached dep NOT loaded from _libs (a partial/namespace dev copy)
    # plus its submodules, so the next import resolves cleanly to _libs.
    import importlib
    for pkg in _BUNDLED_DEPS:
        m = sys.modules.get(pkg)
        f = getattr(m, "__file__", None) or ""
        if m is not None and not f.startswith(libs):
            for name in [n for n in list(sys.modules)
                         if n == pkg or n.startswith(pkg + ".")]:
                del sys.modules[name]
    importlib.invalidate_caches()


_ensure_wheels()

from . import ui            # noqa: E402
from .src import ops        # noqa: E402


def register() -> None:
    WindowManager.manekko_running = BoolProperty(
        name="Manekko Running", default=False, options={"SKIP_SAVE"})
    WindowManager.manekko_recording = BoolProperty(
        name="Manekko Recording", default=False, options={"SKIP_SAVE"})
    # Controller-MENU workflow (v0.2.0): left MENU toggles this mode, right MENU
    # acts within it (calib = immediate calibrate; record = start/punch-in/out).
    WindowManager.manekko_mode = EnumProperty(
        name="Manekko Mode",
        items=[("CALIB", "Calibration", "Right MENU = immediate calibrate"),
               ("RECORD", "Record", "Right MENU = start / punch-in / punch-out")],
        default="CALIB", options={"SKIP_SAVE"})
    # WAV cue played when the timeline crosses frame = fps*10 (preloaded on Start
    # to avoid latency). The performer times their motion to this cue.
    WindowManager.manekko_wav_path = StringProperty(
        name="Cue WAV", subtype="FILE_PATH", default="")
    # +/- this many frames are crossfaded at each punch-in/out boundary in the
    # post-recording batch smooth (12 total at the default 6).
    WindowManager.manekko_smooth_frames = IntProperty(
        name="Smooth Frames", default=6, min=0, max=60)
    ops.register()
    ui.register()


def unregister() -> None:
    ui.unregister()
    ops.unregister()
    for attr in ("manekko_running", "manekko_recording", "manekko_mode",
                 "manekko_wav_path", "manekko_smooth_frames"):
        try:
            delattr(WindowManager, attr)
        except Exception:
            pass
