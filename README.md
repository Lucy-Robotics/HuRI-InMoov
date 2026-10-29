# HuRI-inmoov: HuRI on Lucy robots

The bridge between [Lucy](https://github.com/Lucy-Robotics/lucy_ws) robots
(InMoov, SO-ARM101/LeRobot, any robot package Lucy runs on ROS 2) and
[HuRI](https://github.com/Lucy-Robotics/huri-core), the realtime
conversational pipeline: the robot listens through Lucy's microphone topic,
HuRI answers, and the answer plays on Lucy's speaker topic.

`huri-lucy-bridge` is a client of both sides, like the
[Lucy control panel](https://github.com/Lucy-Robotics/lucy_control_panel): it
talks to the robot through rosbridge (no ROS install), registers in Lucy's
client registry by heartbeat, and opens one HuRI session. It subscribes to ROS
topics and sends them to HuRI, and subscribes to HuRI topics and publishes them
on ROS. Nothing in it, or in its config, names a robot.

> **Status: 0.1.0.** Voice works on every Lucy robot. The bridge discovers the
> robot and what it can do; the routes that move it (gestures, speech motion)
> are next. Not yet run against a real Lucy rosbridge: the tests use a fake one
> speaking the same protocol, Lucy's real hardware configs, and HuRI's real
> session endpoint.

## Architecture

```
 Lucy machine (lucy_ws, pixi run core)                    HuRI server (huri-core[server])
 ┌───────────────────────────────────────────────┐       ┌──────────────────────────────┐
 │ control panel mic ─┐                          │       │                              │
 │ audio_capturer ────┴─> /mic_audio ─┐          │       │ vad > stt > tag/emo > qag    │
 │                                    │          │       │     > rag > tts (> mov)      │
 │ /config/get (the robot) ──────┐    │          │ GET   │                              │
 │                   rosbridge   v    v          │/modules                              │
 │                         huri-lucy-bridge ─────┼──────>│ huri.audio.in  (int16 PCM)   │
 │                           │  profile    <─────┼───────│ huri.audio.out (float32)     │
 │                           │  heartbeat        │  ws   └──────────────────────────────┘
 │                           v                   │
 │ control panel speakers <─ /audio              │
 │ audio_player_node <───────┘                   │
 └───────────────────────────────────────────────┘
```

The bridge works with any robot by layering what it knows:

1. **What every Lucy robot shares**: audio (`/mic_audio`, `/audio`), presence
   (`/lucy/client_heartbeat`), control (`/lucy/control`), cameras. Routes on
   these know nothing of the robot: voice works everywhere.
2. **What Lucy says about the robot**, read at every connection and never
   configured: `/config/get` gives the robot package (`inmoov_urdf`,
   `so_arm101_urdf`...), its JointTrajectory controllers and their joint order,
   and each joint's limits and rest, which the bridge converts to URDF radians
   (`huri_lucy_bridge/robot.py`, parsed as lucy_cli and the panel parse it).
   Lucy starts its config services after rosbridge: the bridge waits for them
   up to `lucy.discovery_timeout_s` (30 s), then bridges voice only.
3. **What HuRI's output does on that robot**: its *embodiment profile*, the
   only robot-specific part, and data (`huri_lucy_bridge/profiles/`). See
   [Robots](#robots).

The HuRI server stays robot-agnostic: it emits speech and SMPL-X human motion,
and the bridge adapts them to the robot it finds. No module per robot on the
server, no robot calibration there.

## Routes

One route per HuRI topic; the YAML's `send` and `subscribe` choose which run.

| ROS (Lucy) | | HuRI | Needs | Conversion |
|---|---|---|---|---|
| `/mic_audio` `audio_common_msgs/msg/AudioStamped` | -> | `huri.audio.in` (send) | - | any PortAudio format and channel count -> int16 mono at `sample_rate`, resampled (linear), in frames of exactly `frame_duration` (webrtcvad takes 10, 20 or 30 ms) |
| `/audio` `audio_common_msgs/msg/AudioStamped` | <- | `huri.audio.out` (subscribe) | - | float32 mono at the TTS rate -> int16 mono at the same rate, `speaker_chunk` samples per message, stamped along the audio (the player and the panel resample to their device) |

The topics and message type are the contract of the control panel's
`AudioBridgeHandler` and of lucy_ros_api's `CONTROL_PANEL_AUDIO.md`: the panel
publishes the browser mic on `/mic_audio` (16 kHz mono int16, 1024 samples per
message, no resampling needed) and plays `/audio`.

A route acting on the robot may need a capability of its profile (*Needs*).
The YAML describes the richest session; for a robot without the capability,
the bridge leaves the route's topic out of the session, and the modules that
only fed it: a robot without arms never runs `mov` on the server's GPU. Which
module feeds what comes from the server's `GET /modules`; a server that does
not answer it only gets no pruning.

## Robots

| | InMoov (i1, i2) | SO-ARM101 (LeRobot) | Any other robot package |
|---|---|---|---|
| Voice | yes | yes | yes |
| Profile | [`inmoov.yaml`](bridge/huri_lucy_bridge/profiles/inmoov.yaml) | [`so_arm101.yaml`](bridge/huri_lucy_bridge/profiles/so_arm101.yaml) (experimental) | none: voice only |
| `gestures`: SMPL-X -> joints | head, torso, both arms | one arm, as the speaker's right arm | - |
| `speech_motion`: a joint that moves while speaking | the jaw | the gripper | - |

A profile, keyed by robot package, says how HuRI's output moves the robot's
URDF joints, in radians:

```yaml
version: 1
robot_packages: [inmoov_urdf, inmoov_i2_urdf]
gestures:                         # capability `gestures`
  max_speed: 1.57                 # rad/s, default of the joints below
  joints:
    i01.head.rothead_link_joint:
      sources: [neck.y, head.y]   # SMPL-X joint.axis, summed
      scale: 1.0                  # rad of joint per rad of sources
      ref: 0.0                    # rad of sources mapping to the joint's rest
speech:                           # capability `speech_motion`
  joint: i01.head.jaw_link_joint
  open: 0.26                      # rad from rest at full voice
```

```
angle = rest + scale * (sum of the sources - ref)
      -> clamped to the joint's limits
      -> moved at most max_speed / fps from the previous frame
```

Rests and limits are not in the profile: they are the robot's, from Lucy, so a
recalibration in the panel needs no profile change. The source is the
component of a SMPL-X joint's axis-angle vector on one axis (SMPL-X zero is the
T-pose: y up, x to the body's left, z forward): a first-order Euler angle,
exact for single-axis rotations, wrong for large combined ones (a proper
swing-twist decomposition is future work). At each connection a profile is
checked against the robot Lucy describes: the joints the robot lacks are left
out with a warning, so one profile serves the variants of a model.

The bridge picks the packaged profile of the robot package; `lucy.profile` in
its YAML names another file, or `none`. **To support a new robot**: write its
profile in `huri_lucy_bridge/profiles/` (one file, its `robot_packages`), and
check it against the robot's hardware config the way
`bridge/tests/test_profiles.py` checks InMoov's and SO-ARM101's (copy its
`active.yaml` to `bridge/tests/fixtures/`). In time profiles belong in the
robot packages themselves, served by Lucy, so a robot's authors own its
embodiment.

The InMoov profile is the former server-side `inmoov` module's `joints.toml`
(servo degrees), ported to Lucy's URDF joints. Like it, no sign has been
checked on a robot.

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
side; it may keep state (the audio routes regroup samples). Register it in
`ROS_TO_HURI` or `HURI_TO_ROS` (`huri_lucy_bridge/app.py`), keyed by its HuRI
topic, with the capability it needs (`Outbound(factory, requires=...)`), and
list that topic in the YAML's `send` or `subscribe`. Its factory gets the
robot's resolved capabilities (the joints, their rest and limits).

Next: `gestures` (`huri.motion` -> `huri_lucy_bridge.gestures.Retargeter` ->
each controller's `trajectory_msgs/msg/JointTrajectory`, joints in the
controller's order) and `speech_motion` (`huri.audio.out`'s loudness -> the
jaw or gripper). Both move hardware: they take `/lucy/control` first, and
stop when another client (the panel's operator) takes it.

## Development

```sh
uv venv && . .venv/bin/activate                 # or python -m venv .venv
uv pip install "huri-core[server,client] @ git+https://github.com/Lucy-Robotics/huri-core.git@0.4.0" \
  "huri-events @ git+https://github.com/Lucy-Robotics/huri-modules.git@dev#subdirectory=packages/huri-events"
uv pip install -e "./bridge[dev]"
pytest && black --check . && isort --check-only . && flake8 && mypy bridge/huri_lucy_bridge
```

CI (`.github/workflows/ci.yml`) does the same on Python 3.10-3.12.
`bridge/tests/test_app.py` runs the bridge between a fake rosbridge
(`bridge/tests/bridge_fakes.py`: the v2 ops, service calls such as Lucy's
`/config/get`, and rosbridge's error status for a message type it cannot
import) and HuRI's real `/session` and `/modules` endpoints, in-process
(`[server]` is for that). `bridge/tests/fixtures/` holds Lucy's real hardware
configs.

## TODO

- [ ] The `gestures` and `speech_motion` routes, with `/lucy/control`.
- [ ] Verify every sign of the InMoov and SO-ARM101 profiles with small
      single-joint motions and a low `max_speed`.
- [ ] Finger retargeting from the SMPL-X hand joints (25-54).
- [ ] Fit `scale`/`ref` of the arms against recorded EMAGE motions.
- [ ] Profiles in the robot packages, served by Lucy; the panel's poses and
      animations (today in the browser's localStorage) there too, as a
      vocabulary of named motions.

## Versions and compatibility

| HuRI-inmoov | huri-core | huri-modules (huri-events) |
|---|---|---|
| 0.1.x | 0.4.x | 0.1.x |

Dependency ranges: `huri-core>=0.4,<0.5`, `huri-events>=0.1,<0.2`. The bridge
speaks rosbridge's v2 protocol, as lucy_ws's `rosbridge_websocket` (ROS 2
Jazzy) serves it, and reads Lucy's hardware config `version: 1`.

## License

GPL-3.0, like HuRI and Lucy: see [LICENSE](LICENSE).
