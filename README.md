# HuRI-inmoov

[InMoov](https://inmoov.fr) robot integration for
[HuRI](https://github.com/Lucy-Robotics/huri-core#huri-ecosystem): the robot listens,
answers with HuRI's voice pipeline, and gestures while it speaks. HuRI's `mov`
module generates SMPL-X gestures from the synthesized speech; this repository
turns them into InMoov servo angles on the server, and drives the servos,
safely, on the robot.

> **Status: scaffold (0.1.0).** The retargeting is a first-order
> approximation, the ranges are uncalibrated defaults and the fingers do not
> move yet. Read [Safety](#safety) before powering a robot.

## Architecture

```
 robot computer (huri-inmoov-robot)                 HuRI server (huri-core[server])
 ┌────────────────────────────────┐   websocket   ┌──────────────────────────────────────────┐
 │ mic ──────── huri.audio.in ────┼──────────────>│ vad > stt > tag/emo > qag > rag > tts    │
 │ speaker <─── huri.audio.out ───┼<──────────────│                     │ huri.audio.out     │
 │                                │               │                     v                    │
 │ Player (frames on pts)         │               │                    mov (EMAGE, GPU)      │
 │   v                            │               │                     │ huri.motion (SMPL-X)│
 │ SafetyController               │               │                     v                    │
 │   limits, speed, watchdog,     │<──────────────┼─ inmoov.servo_angles ─ inmoov (this repo)│
 │   e-stop                       │               │   (degrees, 24 servos)  actuation module │
 │   v                            │               └──────────────────────────────────────────┘
 │ SerialDriver ── "S<id>:<deg>" ─┼──> Arduino (arduino/huri_inmoov_servos) ──> servos
 └────────────────────────────────┘
```

Three distributions, one repository:

| Path | Distribution (import) | Installed on | What it is |
|---|---|---|---|
| `module/` | `huri-module-inmoov` (`huri_inmoov`) | the server | the `inmoov` module: `huri.motion` (Motion) -> `inmoov.servo_angles` (ServoAngles); retargeting config in `huri_inmoov/joints.toml`, manifest in `huri_inmoov/huri_module.toml` |
| `events/` | `huri-events-inmoov` (`huri_events_inmoov`) | both | the `ServoAngles` event and its wire format, entry point `inmoov.servo_angles` |
| `robot/` | `huri-inmoov-robot` (`huri_inmoov_robot`) | the robot | the client (console script `huri-inmoov-robot`), its safety layer, the servo drivers and the Arduino sketch (`robot/arduino/`) |

## Install

`<core-ref>` and `<modules-ref>` are git refs of huri-core and huri-modules:
`dev` for now, release tags (`0.4.0`, `0.1.0`) once they exist (see
[Versions](#versions-and-compatibility)). `<ref>` is a ref of this repository.
The events package is on no package index: install it in the same pip call as
the package that needs it.

### On the HuRI server

Next to huri-core and the curated modules the pipeline uses (`vad`, `stt`,
..., `mov`: see the
[huri-modules README](https://github.com/Lucy-Robotics/huri-modules#readme)):

```sh
python -m pip install \
  "huri-core[server] @ git+https://github.com/Lucy-Robotics/huri-core.git@<core-ref>" \
  "huri-events @ git+https://github.com/Lucy-Robotics/huri-modules.git@<modules-ref>#subdirectory=packages/huri-events" \
  "huri-events-inmoov @ git+https://github.com/Lucy-Robotics/HuRI-inmoov.git@<ref>#subdirectory=events" \
  "huri-module-inmoov @ git+https://github.com/Lucy-Robotics/HuRI-inmoov.git@<ref>#subdirectory=module"
```

Enable it, and opt in to it moving hardware: `inmoov` declares
`actuation = True` (class) and `requires.actuation = true` (manifest), so the
server refuses to load it unless it is listed in **both** variables:

```sh
HURI_MODULES=vad,stt,tag,emo,eag,qag,rag,tts,mov,inmoov
HURI_ACTUATION_MODULES=inmoov
```

The robot's calibration (a joints config) is the operator's choice, never a
client's: `HURI_INMOOV_JOINTS=/etc/huri/inmoov_joints.toml` on the server
(default: the packaged `joints.toml`). It is read once, at startup, and the
server refuses to start if the file is missing, not a regular file, larger
than 64 KiB or invalid. The module takes no argument; a client passing
`joints` gets its session refused.

### On the robot

A client: `huri-core[client]`, no Ray, no module, no GPU.

```sh
python -m pip install \
  "huri-core[client] @ git+https://github.com/Lucy-Robotics/huri-core.git@<core-ref>" \
  "huri-events @ git+https://github.com/Lucy-Robotics/huri-modules.git@<modules-ref>#subdirectory=packages/huri-events" \
  "huri-events-inmoov @ git+https://github.com/Lucy-Robotics/HuRI-inmoov.git@<ref>#subdirectory=events" \
  "huri-inmoov-robot[audio] @ git+https://github.com/Lucy-Robotics/HuRI-inmoov.git@<ref>#subdirectory=robot"

huri-inmoov-robot --config robot/config/robot_example.yaml --fake   # dry run: no servo moves
huri-inmoov-robot --config my_robot.yaml                             # the real thing
```

The config is a HuRI client YAML (`huri_url`, `modules`, `send`, `subscribe`)
plus a `robot:` section the server never sees: driver and serial port, servo
-> board channel map, watchdog, e-stop action, limit overrides, audio. See
[`robot/config/robot_example.yaml`](robot/config/robot_example.yaml). Without
`[audio]` (sounddevice), drop `huri.audio.in` from `send` and set
`robot.audio.speaker: false`.

Flash [`robot/arduino/huri_inmoov_servos`](robot/arduino/huri_inmoov_servos/huri_inmoov_servos.ino)
after filling its pin and range tables. Serial protocol (115200 baud, one
ASCII line per command): `S<id>:<deg>` moves channel `<id>`, `D` detaches every
servo, `A` attaches them again.

### Development

```sh
uv venv && . .venv/bin/activate                 # or python -m venv .venv
uv pip install -e "<huri-core checkout>[server,client]" -e "<huri-modules checkout>/packages/huri-events"
uv pip install -e ./events -e "./module[dev]" -e ./robot
pytest && black --check . && isort --check-only . && flake8
```

CI (`.github/workflows/ci.yml`) does the same from git refs
(`HURI_CORE_REF`, `HURI_MODULES_REF`, `dev` until the release tags exist) on
Python 3.10-3.12. `robot/tests/test_end_to_end.py` runs the robot client
against HuRI's real `/session` endpoint with the `inmoov` module loaded.

## Retargeting (server)

For each servo of the layout, `joints.toml` lists SMPL-X joint components
(joint name or index 0-54, axis x/y/z of its axis-angle vector), a `scale`, a
`ref` (the SMPL-X angle mapping to rest; the "offset"), `rest`, `[min, max]`
and an optional `max_speed`:

```
angle = rest + scale * (sum of source angles in degrees - ref)
      -> clamped to [min, max]
      -> moved at most max_speed / fps from the previous frame
```

The module keeps the last angles of the session, so the speed limit holds
across Motion chunks; it starts at rest. `fps` and `pts` pass through: one
ServoAngles per Motion, frame for frame.

Joint set (layout 1): `head.neck` (pitch: neck + head x), `head.rothead`
(yaw: neck + head y), `head.jaw` (optional, `scale = 0` keeps it at rest),
`torso.midStom` (yaw: spine1-3 y), and per arm `omoplate` (abduction, shoulder
z), `shoulder` (flexion, shoulder y), `rotate` (upper-arm roll, shoulder x),
`bicep` (elbow flexion, elbow y). Hands (`thumb`, `index`, `majeure`,
`ringFinger`, `pinky`, `wrist`): **TODO**, held at rest.

Assumptions (all to verify on a robot):

- SMPL-X zero is the T-pose: y up, x to the body's left, z forward; local
  joint rotations. The component of the axis-angle vector approximates the
  Euler angle about that axis: exact for single-axis rotations, wrong for
  large combined ones (a proper swing-twist decomposition is future work).
- The arms' `omoplate` uses `ref = ∓80°` so that an arm lowered along the body
  (EMAGE's usual pose) is the robot's rest, not the T-pose.
- Signs (`scale`) follow the directions written in `joints.toml`; a servo
  mounted the other way needs its sign flipped.
- Ranges and rest angles are the commonly shipped defaults of MyRobotLab's
  InMoov2 services (head neck/rothead 20-160 rest 90, jaw 10-25, omoplate
  10-80 rest 10, shoulder 0-180 rest 30, rotate 40-180 rest 90, bicep 5-90 rest
  5, fingers ~2-170); midStom is narrowed to 60-120 on purpose. They are a
  starting point, not your robot's calibration.

## The `inmoov.servo_angles` event

`huri_events_inmoov.ServoAngles`: `angles` (n_frames x n_servos, degrees),
`fps`, `pts` (matches the Motion/Audio it comes from), `layout`. Wire
(binary): a 24-byte big-endian header `>dIIII` (pts f64, fps u32, n_frames
u32, n_servos u32, layout u32), then n_frames x n_servos little-endian float32
angles, frame by frame. Servo names are not sent: they are fixed by the layout
version (`SERVO_LAYOUTS[1]`, 24 servos, `SERVO_NAMES`). A released layout never
changes; a new servo set is a new layout, and a decoder refuses a layout it
does not know.

## Safety

The robot never trusts the server. Every angle goes through
`SafetyController` before the driver:

- **Hard limits**: the robot's own per-servo `[min, max]`
  (`huri_inmoov_robot/limits.py`, overridden by `robot.limits`), independent
  of the server's joints config, so a wrong server configuration cannot widen
  them. The Arduino sketch clamps once more to its own table.
- **Max speed**: each servo moves at most `max_speed` deg/s of real time
  between commands; a pause never allows a jump (dt is capped at 0.1 s).
- **Watchdog**: no frame for `watchdog_ms` (1500 ms by default) -> `rest` (walk
  back to rest at the speed limit) or `hold`. Gestures pause between
  sentences: a shorter watchdog walks the arms to rest and back in every
  pause, a longer one leaves them up longer after a fault. The Arduino lights its LED after
  1 s without a command, and never moves a servo on its own.
- **Driver fault**: a serial write blocked longer than `write_timeout`
  (50 ms; the board stopped reading) latches the e-stop and sends nothing
  more. If the player or watchdog task dies (e.g. the USB cable comes out),
  the client e-stops, leaves the session and closes the driver.
- **E-stop** (latched until reset): `<Enter>` in the terminal or
  `kill -USR1 <pid>`; `r` + `<Enter>` resets. `estop_action: hold` keeps the
  torque (the robot freezes); `detach` removes it, and **an InMoov's arms then
  fall**: use it only when that is safe.
- **Shutdown** (Ctrl+C, SIGTERM, session closed): the robot walks back to rest
  at the speed limit, then the driver closes.
- The robot is assumed at rest at start (the sketch attaches every servo at its
  rest angle).

A software e-stop is not a hardware one: wire a physical switch that cuts the
servos' power, and keep it in reach. First runs: `--fake`, then one servo
connected, small ranges, low `max_speed`.

## Calibration TODOs

- [ ] Measure each servo's safe range and rest angle; write them in the
      server's `joints.toml` (a copy pointed to by `HURI_INMOOV_JOINTS`), the robot's
      `robot.limits` and the sketch's tables.
- [ ] Check every `scale` sign with small single-joint motions.
- [ ] Fit `scale`/`ref` of the arms against recorded EMAGE motions.
- [ ] Finger retargeting from the SMPL-X hand joints (25-54).
- [ ] Two boards (InMoov's left/right Arduinos): one SerialDriver per port.
- [ ] Anchor the playback clock on the speaker's position (the player
      schedules frames on `pts` from the wall-clock time of each utterance's
      first frame, and drops only frames already late).

## Versions and compatibility

| HuRI-inmoov | huri-core | huri-modules (huri-events) |
|---|---|---|
| 0.1.x | 0.4.x | 0.1.x |

The three packages of this repository are released together, with the same
version. Dependency ranges: `huri-core>=0.4.0.dev0,<0.5`, `huri-events>=0.1.0.dev0,<0.2`,
`huri-events-inmoov>=0.1,<0.2`.

## License

GPL-3.0, like HuRI: see [LICENSE](LICENSE).
