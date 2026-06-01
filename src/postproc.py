"""Post-recording batch smoothing of a captured take (v0.2.0).

All smoothing happens here, *after* recording stops — there is no real-time
ease-in (the live capture writes raw FK keys frame-by-frame). Two operations,
both operating directly on the active Action's fcurves with a smoothstep weight
(3t^2 - 2t^3), and both covering position channels as well as rotation:

1. **Front ramp** (first take only): the very start must be a clean rest pose so
   Marvelous Designer can initialise the garment, then ease into the motion.
   * frames ``1 .. rest_end`` (1 s)         -> forced to the rest pose
   * frames ``rest_end .. ramp_end`` (5 s)  -> blend rest -> recorded
   * frames after ``ramp_end``              -> recorded as-is
   This only fires when recording started within the rest window
   (``punch_in <= rest_end``); a punch-in retake further in skips it.

2. **Punch-in / punch-out crossfade** (retakes): around each boundary, +/-N
   frames (``smooth_frames``, default 6 -> 12 total) crossfade the previous
   take (``old_seg``, sampled before it was overwritten) with the new take so
   the seam is continuous. Skipped when there is no previous take to blend with.

Quaternion channels are blended component-wise (not slerp): the per-fcurve model
has no access to the sibling components, and near rest / small seams the error is
negligible. Acceptable for this indie tool (same "experiment & adjust" stance as
the other knobs).
"""
from __future__ import annotations


def smoothstep(x: float) -> float:
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    return x * x * (3.0 - 2.0 * x)


def _set_key(fc, frame: int, value: float) -> None:
    """Set (or insert) a keyframe value at an integer frame on one fcurve."""
    for kp in fc.keyframe_points:
        if abs(kp.co[0] - frame) < 1e-6:
            kp.co[1] = value
            return
    fc.keyframe_points.insert(frame, value, options={"FAST"})


def sample_segment(fcurves, start_frame: int) -> dict:
    """Sample every fcurve in ``fcurves`` from ``start_frame`` to its last key,
    BEFORE the live capture overwrites it. Returns ``{(data_path, idx): {f: v}}``.

    Used to preserve the previous take across the to-be-recorded region so the
    punch-in/out crossfade has the 'old' curve to blend against (the recording
    overwrites those frames in place). ``fcurves`` is gathered by the caller
    (legacy ``action.fcurves`` or, on Blender 4.4+, the slot's channelbag)."""
    out: dict = {}
    for fc in fcurves:
        kps = fc.keyframe_points
        if not kps:
            continue
        last = int(round(kps[-1].co[0]))
        if last < start_frame:
            continue
        seg = {f: fc.evaluate(f) for f in range(start_frame, last + 1)}
        out[(fc.data_path, fc.array_index)] = seg
    return out


def rest_channel_values(arm, driver, fcurves) -> dict:
    """Rest-pose value of every keyed channel (for the front ramp).

    Drives the rig to its rest (A) pose with open hands, then resolves each
    fcurve's property value. Returns ``{(data_path, idx): value}``."""
    import bpy

    from . import apply as _apply

    driver.solver.reset_to_rest()
    _apply.apply_pose(arm, driver.rm, driver.solver.configuration,
                      fingers={"hand_l": 0.0, "hand_r": 0.0},
                      finger_names=driver.finger_names)
    bpy.context.view_layer.update()

    out: dict = {}
    for fc in fcurves:
        try:
            prop = arm.path_resolve(fc.data_path)
            val = prop[fc.array_index]
        except Exception:  # noqa: BLE001
            continue
        out[(fc.data_path, fc.array_index)] = float(val)
    return out


def smooth_take(arm, driver, *, fcurves, fps: float, punch_in, punch_out,
                old_seg: dict, smooth_frames: int) -> None:
    """Batch-smooth the recorded take in place. See module docstring."""
    if not fcurves:
        return
    rest_end = max(1, int(round(fps * 1.0)))     # 1 s -> end of forced rest
    ramp_end = max(rest_end + 1, int(round(fps * 5.0)))  # 5 s -> production zone
    n = max(0, int(smooth_frames))

    front = punch_in is not None and punch_in <= rest_end
    rest_vals = rest_channel_values(arm, driver, fcurves) if front else {}

    for fc in fcurves:
        key = (fc.data_path, fc.array_index)
        seg = old_seg.get(key)

        # 1) front ramp (first take): rest hold then ease rest -> recorded
        if front and key in rest_vals:
            rv = rest_vals[key]
            ramp_samples = {f: fc.evaluate(f)
                            for f in range(rest_end, ramp_end + 1)}
            for f in range(1, rest_end + 1):
                _set_key(fc, f, rv)
            span = float(ramp_end - rest_end)
            for f in range(rest_end, ramp_end + 1):
                w = smoothstep((f - rest_end) / span)
                _set_key(fc, f, rv * (1.0 - w) + ramp_samples[f] * w)

        # 2) punch-in crossfade: old -> new across [in-n, in+n]
        if n > 0 and punch_in is not None and seg:
            new_at_in = fc.evaluate(punch_in)
            samples = {}
            for f in range(punch_in - n, punch_in + n + 1):
                oldv = seg.get(f, fc.evaluate(f))
                newv = fc.evaluate(f) if f >= punch_in else new_at_in
                w = smoothstep((f - (punch_in - n)) / float(2 * n))
                samples[f] = oldv * (1.0 - w) + newv * w
            for f, v in samples.items():
                _set_key(fc, f, v)

        # 3) punch-out crossfade: new -> old across [out-n, out+n]
        if n > 0 and punch_out is not None and seg:
            new_at_out = fc.evaluate(punch_out)
            samples = {}
            for f in range(punch_out - n, punch_out + n + 1):
                oldv = seg.get(f, fc.evaluate(f))
                newv = fc.evaluate(f) if f <= punch_out else new_at_out
                w = smoothstep((f - (punch_out - n)) / float(2 * n))
                samples[f] = newv * (1.0 - w) + oldv * w
            for f, v in samples.items():
                _set_key(fc, f, v)

        fc.update()
