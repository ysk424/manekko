# Controller-MENU recording workflow (v0.2.2 — live-verified)

Verified on hardware 2026-06-02: punch-in/out mocap recording works; the default
6-frame crossfade "looks quite natural" (owner). Audio cue + stop-on-stop work.
The drift caveat below did not bite in practice.

Adds a controller-driven record/calibrate workflow on top of the N-panel
buttons (the buttons remain as a fallback). Implemented in `src/ops.py`
(modal state machine + audio), `src/postproc.py` (batch smoothing), `ui.py`
(mode display + WAV + smooth-frames fields), `__init__.py` (properties).

## Buttons (legacy `getControllerState`, ApplicationMenu bit `0x2`)

Left controller = `palm_l`, right = `palm_r`. Rising-edge detected each tick.

- **Left MENU** = mode toggle: `CALIBRATION` (default) ⇄ `RECORD`.
  Entering RECORD rewinds the timeline to frame 1 and re-arms the right cycle
  (this toggle is also the *retake reset*: tap calib→record to rewind & re-arm).
- **Right MENU**:
  - CALIBRATION mode → **immediate** A-pose calibrate + JSON save + beep
    (the old 5 s countdown is gone for this path; the N-panel Calibrate button
    still uses the 5 s countdown).
  - RECORD mode, 3-press cycle:
    1. start playback from frame 1 (plays the existing take; no live drive)
    2. **punch-in**: begin live drive + record at the current frame
    3. **punch-out**: stop; run the batch smooth

The current mode is shown in the N-panel ("Controller MENU" box).

## Timeline (fps-aware; numbers shown at 24 fps)

- frames `1 .. fps*1` (1 s): rest pose — clean start for Marvelous Designer cloth init
- frames `fps*1 .. fps*5` (1–5 s): smooth ramp rest → motion
- frames `fps*5 ..` (5 s+): production motion (本番)
- frame `fps*10` (240): **cue WAV** plays (preloaded `aud.Sound`, no latency).
  The performer times their motion to this cue. Fires once per playback, on the
  crossing, during playback or recording.

## Batch smoothing (`src/postproc.py`) — ALL smoothing is post-record

There is **no real-time ease-in**; live capture writes raw FK keys. On
punch-out (or leaving record mode mid-take) `smooth_take` runs:

1. **Front ramp** (first take only, i.e. `punch_in <= fps*1`): force frames
   `1..fps*1` to the rest pose, then smoothstep-blend rest → recorded over
   `fps*1 .. fps*5`. This is what gives MD a clean garment-init frame.
2. **Punch-in / punch-out crossfade** (retakes): ±`smooth_frames` (N-panel,
   default 6 → 12 total) around each boundary, smoothstep-crossfade the previous
   take against the new one. The previous take is sampled into `old_seg` at
   punch-in *before* the recording overwrites it (so the seam has the old curve
   to blend against).

Both cover **position** channels as well as rotation. Quaternions are blended
component-wise (no slerp) — fine near rest / small seams; revisit if a seam
visibly wobbles.

## Known caveats to verify live

- **Audio vs. frame drift**: the modal advances one frame per timer tick; if the
  solve can't keep real-time, the `aud` (wall-clock) cue and the recorded frame
  counter diverge, so the performer's music-timed motion may land on the wrong
  frames. If this bites, switch frame advance to elapsed-wall-time based.
- First take in the "playing" (pre-punch-in) state shows the existing take only;
  with no take yet the character is static until punch-in (matches the spec).
- Component-wise quaternion blend (see above).
