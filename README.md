# HuRI-inmoov

The bridge between [Lucy](https://github.com/Lucy-Robotics/lucy_ws) robots
(the InMoov humanoid, the SO-ARM101... on ROS 2) and
[HuRI](https://github.com/Lucy-Robotics/huri-core), the realtime
conversational pipeline: the robot listens through Lucy's microphone topic,
HuRI answers, and the answer plays on Lucy's speaker topic.

`huri-lucy-bridge` is a client of both sides, like the
[Lucy control panel](https://github.com/Lucy-Robotics/lucy_control_panel): it
talks to the robot through rosbridge (no ROS install), registers in Lucy's
client registry by heartbeat, and opens one HuRI session. It subscribes to ROS
topics and sends them to HuRI, and subscribes to HuRI topics and publishes them
on ROS.

> **Status: 0.1.0, audio only.** Microphone and speech are bridged; motion
> (HuRI's gestures to Lucy's joint trajectories) is the next route. Not yet
> run against a real Lucy rosbridge: the tests use a fake one speaking the same
> protocol, and HuRI's real session endpoint.

## Architecture

```
 Lucy machine (lucy_ws, pixi run core)                    HuRI server (huri-core[server])
 ┌───────────────────────────────────────────────┐       ┌──────────────────────────────┐
 │ control panel mic ─┐                          │       │                              │
 │ audio_capturer ────┴─> /mic_audio ─┐          │       │ vad > stt > tag/emo > qag    │
 │                                    │          │       │     > rag > tts              │
 │                   rosbridge :9090  │          │       │                              │
 │                                    v          │  ws   │                              │
 │                         huri-lucy-bridge ─────┼──────>│ huri.audio.in  (int16 PCM)   │
 │                           │  heartbeat  <─────┼───────│ huri.audio.out (float32)     │
 │                           │  /lucy/client_heartbeat   └──────────────────────────────┘
 │                           v                   │
 │ control panel speakers <─ /audio              │
 │ audio_player_node <───────┘                   │
 └───────────────────────────────────────────────┘
```

| Path | Distribution (import) | Installed on | What it is |
|---|---|---|---|
| `bridge/` | `huri-lucy-bridge` (`huri_lucy_bridge`) | anywhere that reaches Lucy's rosbridge and the HuRI server | the bridge (console script `huri-lucy-bridge`) |
| `module/` | `huri-module-inmoov` (`huri_inmoov`) | the HuRI server | the `inmoov` module: `huri.motion` (SMPL-X) -> `inmoov.servo_angles`. Not bridged yet: the motion route will need it, or a retargeting to Lucy's URDF joints |
| `events/` | `huri-events-inmoov` (`huri_events_inmoov`) | the HuRI server (and the bridge, with the motion route) | the `ServoAngles` event, entry point `inmoov.servo_angles` |

The former robot-side client (`robot/`, `huri-inmoov-robot`: serial servo
driver, Arduino sketch, its own safety layer) is gone: Lucy owns the hardware now (ros2_control, the
MCU firmware, the joint limits of its hardware config), and nothing else may
drive the servos.

## Routes

One route per HuRI topic; the YAML's `send` and `subscribe` choose which run.

| ROS (Lucy) | | HuRI | Conversion |
|---|---|---|---|
| `/mic_audio` `audio_common_msgs/msg/AudioStamped` | -> | `huri.audio.in` (send) | any PortAudio format and channel count -> int16 mono at `sample_rate`, resampled (linear), in frames of exactly `frame_duration` (webrtcvad takes 10, 20 or 30 ms) |
| `/audio` `audio_common_msgs/msg/AudioStamped` | <- | `huri.audio.out` (subscribe) | float32 mono at the TTS rate -> int16 mono at the same rate, `speaker_chunk` samples per message, stamped along the audio (the player and the panel resample to their device) |

The topics and message type are the contract of the control panel's
`AudioBridgeHandler` and of lucy_ros_api's `CONTROL_PANEL_AUDIO.md`: the panel
publishes the browser mic on `/mic_audio` (16 kHz mono int16, 1024 samples per
message, no resampling needed) and plays `/audio`.

## On the Lucy side

1. **Lucy's stack**, with rosbridge: `pixi run core` in lucy_ws (or the
   launcher's Core). rosbridge listens on `ws://<lucy machine>:9090`.
2. **`audio_common_msgs` where rosbridge runs.** rosbridge (de)serializes
   `AudioStamped` by importing its package, and lucy_ws's pixi environment does
   not ship it today (lucy_bringup declares `audio_common`, the pixi lock has no
   audio_common package): build [audio_common](https://github.com/mgonzs13/audio_common)
   in `lucy_ws/src` with the workspace. Without it, rosbridge answers the
   bridge's advertise and subscribe with an error status naming the type, which
   the bridge logs (`rosbridge: ...`), and no audio flows. The control panel's own mic and speakers need it as much.
3. **A microphone and a speaker**, one of:
   - the control panel: connect, turn **Mic** and **Speakers** on (the browser
     is the robot's ears and mouth; use headphones);
   - the robot's own devices: lucy_bringup does not start the audio nodes
     today, so run them, the capturer remapped to `/mic_audio`:
     ```sh
     ros2 run audio_common audio_capturer_node --ros-args -r audio:=/mic_audio
     ros2 run audio_common audio_player_node      # plays /audio
     ```
     Unremapped, the capturer publishes on `/audio` and the player plays the
     microphone.

   Only one publisher of `/mic_audio` at a time: two sources mix into noise.

The bridge takes no control (`/lucy/control`): it moves nothing, and the panel
keeps control. It shows in the panel's connected-client count.

## Install and run

`<core-ref>` and `<modules-ref>` are git refs of huri-core and huri-modules:
`0.4.0` and `dev` for now (see [Versions](#versions-and-compatibility)).
`<ref>` is a ref of this repository.

```sh
python -m venv ~/huri-bridge && . ~/huri-bridge/bin/activate
python -m pip install \
  "huri-core[client] @ git+https://github.com/Lucy-Robotics/huri-core.git@<core-ref>" \
  "huri-events @ git+https://github.com/Lucy-Robotics/huri-modules.git@<modules-ref>#subdirectory=packages/huri-events" \
  "huri-lucy-bridge @ git+https://github.com/Lucy-Robotics/HuRI-inmoov.git@<ref>#subdirectory=bridge"

huri-lucy-bridge --config bridge/config/bridge_example.yaml
```

A client of HuRI (huri-core[client]: no Ray, no module, no GPU) and of
rosbridge: no ROS, no audio library. It runs on the Lucy machine or any
machine that reaches both servers. The HuRI server serves the voice pipeline
(`HURI_MODULES=vad,stt,tag,emo,eag,qag,rag,tts`: see the
[huri-modules README](https://github.com/Lucy-Robotics/huri-modules#readme)).

The config is a HuRI client YAML (`huri_url`, `modules`, `send`, `subscribe`)
plus a `lucy:` section the server never sees: rosbridge URL, client id prefix,
heartbeat and reconnection periods, audio topics and format. See
[`bridge/config/bridge_example.yaml`](bridge/config/bridge_example.yaml).
When HuRI or rosbridge goes away the bridge reconnects every `reconnect_s`
seconds under the same client id (`0`: it exits); a session HuRI refuses (an
unknown module, a broken pipeline) is not retried.

### From the Lucy launcher

An `interface` entry in lucy_ws's `config/launcher_config.json.local` (a copy
of `launcher_config.json`: the local file replaces it, it is not merged)
starts the bridge in its own tmux window:

```json
{
  "id": "huri_bridge",
  "name": "HuRI bridge",
  "description": "Voice: /mic_audio -> HuRI -> /audio",
  "type": "interface",
  "dependencies": ["core"],
  "conflicts": [],
  "command": "~/huri-bridge/bin/huri-lucy-bridge --config ~/huri-bridge/bridge.yaml",
  "default_on": false
}
```

### Known limits

- **Echo**: there is no echo cancellation. With a speaker next to the
  microphone, HuRI hears its own answers; use headphones on the panel.
- **Resampling** is linear, without an anti-aliasing filter: a 48 kHz robot
  microphone folds some noise into the 16 kHz the vad hears. Publish 16 kHz
  when the source can (the panel does).

## Adding a route

A route (`huri_lucy_bridge/routes.py`) names its ROS topic and type and its
HuRI topic, and converts one message to zero or more messages of the other
side; it may keep state (the audio routes regroup samples). Register its
factory in `ROS_TO_HURI` or `HURI_TO_ROS` (`huri_lucy_bridge/app.py`), keyed
by its HuRI topic, and list that topic in the YAML's `send` or `subscribe`.
Motion is next: `huri.motion` or `inmoov.servo_angles` -> Lucy's
`trajectory_msgs/msg/JointTrajectory` controllers, whose joint names and order
come from Lucy's `/config/get`, never hardcoded, and which need control
(`/lucy/control`) first.

## Development

```sh
uv venv && . .venv/bin/activate                 # or python -m venv .venv
uv pip install "huri-core[server,client] @ git+https://github.com/Lucy-Robotics/huri-core.git@0.4.0" \
  "huri-events @ git+https://github.com/Lucy-Robotics/huri-modules.git@dev#subdirectory=packages/huri-events"
uv pip install -e "./bridge[dev]" -e ./events -e ./module
pytest && black --check . && isort --check-only . && flake8 \
  && mypy bridge/huri_lucy_bridge module/huri_inmoov events/huri_events_inmoov
```

CI (`.github/workflows/ci.yml`) does the same on Python 3.10-3.12.
`bridge/tests/test_app.py` runs the bridge between a fake rosbridge
(`bridge/tests/bridge_fakes.py`: the v2 ops, and rosbridge's error status for a
message type it cannot import) and HuRI's real `/session` endpoint, in-process
(`[server]` is for that, and for the module's tests).

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


## Calibration TODOs (the `inmoov` module)

- [ ] Measure each servo's safe range and rest angle; write them in the
      server's `joints.toml` (a copy pointed to by `HURI_INMOOV_JOINTS`).
- [ ] Check every `scale` sign with small single-joint motions.
- [ ] Fit `scale`/`ref` of the arms against recorded EMAGE motions.
- [ ] Finger retargeting from the SMPL-X hand joints (25-54).
- [ ] Map the servo layout to Lucy's InMoov URDF joints (radians), for the
      motion route.

## Versions and compatibility

| HuRI-inmoov | huri-core | huri-modules (huri-events) |
|---|---|---|
| 0.1.x | 0.4.x | 0.1.x |

The three packages of this repository are released together, with the same
version. Dependency ranges: `huri-core>=0.4,<0.5` (`>=0.4.0.dev0` for the
module and events), `huri-events>=0.1,<0.2`, `huri-events-inmoov>=0.1,<0.2`.
The bridge speaks rosbridge's v2 protocol, as lucy_ws's `rosbridge_websocket`
(ROS 2 Jazzy) serves it.

## License

GPL-3.0, like HuRI and Lucy: see [LICENSE](LICENSE).
