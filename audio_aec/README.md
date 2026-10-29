# audio_aec

## Overview

`audio_aec` provides `aec_node`, an acoustic echo cancellation (AEC) node for
ROS 2. The node removes the sound that the robot's own speaker is playing from
the microphone signal, so the microphone can keep hearing the user during
playback. It subscribes to a microphone topic and to a playback reference,
ideally the `played` topic from `audio_player_node`, and publishes cleaned
mono audio.

The motivating use case is text-to-speech: `tts_node` generates speech that
`audio_player_node` plays through the speaker, and `audio_capturer_node` picks
that speech up again through the microphone. The player publishes the exact
post-resampling, post-channel-conversion frames it writes to the device on
`/audio/played`; feeding that topic to `aec_node` avoids making the AEC model
the player's resampling and device buffering. The same mechanism works for
`music_node` or any other `AudioStamped` source that feeds the speaker.

Echo cancellation is performed by WebRTC AudioProcessing. The node resamples
both inputs to a fixed internal rate, selects reference frames by their
playback timestamps instead of FIFO arrival order, processes 10 ms frames, and
publishes mono output in chunks. The timestamp alignment keeps the AEC delay
stable when the reference and microphone callbacks jitter; the optional
reference lead and wait parameters absorb the normal burstiness of the player.
No existing `audio_common` node needs to be modified; the graph is wired with
topic remapping.

## Install

```shell
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to audio_aec
source install/setup.bash
```

Ubuntu 20.04 through 24.04 ship WebRTC AudioProcessing 0.3.1, while Ubuntu 25.10
and later ship 1.3. The package supports both APIs. It uses 0.3.1 by default
because its high-suppression mode is the best tested configuration for this
pipeline, and falls back to 1.3 automatically if 0.3.1 is unavailable. To try
the AEC3 API when both libraries are installed:

```shell
colcon build --packages-select audio_aec --cmake-args -DAUDIO_AEC_USE_AEC3=ON
```

AEC3 is more conservative in double-talk than the 0.3.1 high-suppression mode
and may leave more residual echo in this application.

## Topics

| Direction | Default topic     | Type                                 | QoS             |
| --------- | ----------------- | ------------------------------------ | --------------- |
| Subscribe | `audio`           | `audio_common_msgs/msg/AudioStamped` | `SensorDataQoS` |
| Subscribe | `audio_reference` | `audio_common_msgs/msg/AudioStamped` | `SensorDataQoS` |
| Publish   | `audio_clean`     | `audio_common_msgs/msg/AudioStamped` | `SensorDataQoS` |

The topic names are fixed defaults that can be changed with standard ROS 2
remappings, for example `-r audio:=/audio/mic -r audio_clean:=/audio/clean`.

Both inputs accept all five PortAudio sample formats supported by
`audio_common_msgs` (`paFloat32`, `paInt32`, `paInt16`, `paInt8`, and `paUInt8`)
at any positive sample rate and channel count. Multi-channel input is downmixed
to mono by averaging the channels. The output is always mono, uses the
configured `format` and `rate`, and carries the microphone input's `frame_id`.
Its stamp is the capture stamp of the first sample in the published chunk, so
the output stays aligned with the original capture timeline even though the node
adds `mic_buffer_ms` of latency.

## Parameters

| Name                       | Type | Default         | Meaning                                                                                     |
| -------------------------- | ---- | --------------- | ------------------------------------------------------------------------------------------- |
| `rate`                     | int  | `16000`         | Internal processing and output rate; one of 8000, 16000, 32000, 48000                       |
| `format`                   | int  | `8` (`paInt16`) | Output PortAudio sample format constant                                                     |
| `mic_buffer_ms`            | int  | `100`           | Additional fixed microphone delay/jitter buffer; at most 500                                |
| `output_chunk_ms`          | int  | `20`            | Duration of each published output message                                                   |
| `stream_delay_ms`          | int  | `0`             | Additional delay hint passed to WebRTC; the effective hint is this plus `reference_lead_ms` |
| `reference_lead_ms`        | int  | `0`             | Select reference audio this far after the microphone timestamp; at most 500                 |
| `reference_wait_ms`        | int  | `100`           | How long a microphone frame may wait for its timestamped reference; between 0 and 1000      |
| `enable_high_pass`         | bool | `true`          | Enable WebRTC's high-pass filter                                                            |
| `enable_noise_suppression` | bool | `false`         | Enable WebRTC noise suppression after AEC                                                   |
| `enable_agc`               | bool | `false`         | Enable WebRTC automatic gain control after AEC                                              |
| `reference_timeout_ms`     | int  | `500`           | After this long without a reference frame, feed silence and reset AEC once                  |

`format` accepts the PortAudio constants `1` (`paFloat32`), `2` (`paInt32`),
`8` (`paInt16`), `16` (`paInt8`), and `32` (`paUInt8`). The default `paInt16`
matches what `audio_capturer_node` publishes and what most speech pipelines
consume. `paFloat32` is available for users who want to keep the processing
domain format.

Validation at startup:

- `rate` must be 8000, 16000, 32000, or 48000.
- `format` must be one of the supported PortAudio constants above.
- `mic_buffer_ms` must be non-negative, a multiple of 10, and at most 500.
- `output_chunk_ms` must be positive and a multiple of 10.
- `stream_delay_ms` must be non-negative.
- `reference_lead_ms` must be between 0 and 500.
- `reference_wait_ms` must be between 0 and 1000.
- `reference_timeout_ms` must be positive.

An invalid value makes the node log an error and fail to start, matching the
other `audio_common` nodes.

## TTS echo demo

Source the workspace in every terminal:

```shell
source ~/ros2_ws/install/setup.bash
```

Run each block in its own terminal.

### 1. Microphone

```shell
ros2 run audio_common audio_capturer_node
```

This publishes the microphone on `/audio`.

### 2. TTS source

```shell
ros2 run audio_common tts_node --ros-args -r audio:=/audio/out
```

`tts_node` publishes on `/audio/out`, so the generated speech does not mix with
the microphone on `/audio`.

### 3. Playback

```shell
ros2 run audio_common audio_player_node --ros-args \
  -r audio:=/audio/out -r played:=/audio/played
```

The player publishes the exact post-resampled frames written to the device on
`/audio/played`; this is the same reference used by the chatbot.

### 4. AEC node

```shell
ros2 run audio_aec aec_node --ros-args -r audio_reference:=/audio/played
```

The microphone and output keep their defaults, `/audio` and `/audio_clean`; the
reference is the player's `/audio/played` topic.

### 5. Monitor the cleaned microphone

```shell
ros2 topic hz /audio_clean --qos-profile sensor_data
ros2 topic echo /audio_clean --qos-profile sensor_data
```

If your `ros2 topic echo` version does not accept `--qos-profile`, use:

```shell
ros2 topic echo /audio_clean --qos-reliability best_effort
```

### 6. Trigger TTS

```shell
ros2 action send_goal /say audio_common_msgs/action/TTS \
  "{text: 'Hello, this is an echo cancellation test.'}"
```

### Optional: listen to the cleaned microphone

Use headphones and a second output device to avoid feedback:

```shell
ros2 run audio_common audio_player_node --ros-args \
  -r __node:=aec_monitor_player -r audio:=audio_clean \
  -p device:=<output_device_index> -p publish_reference:=false
```

The monitor player disables reference publishing so it cannot feed the cleaned
signal back into the AEC.

`audio_capturer_node` keeps publishing on `audio`, which is `aec_node`'s default
microphone topic. `tts_node` publishes on `/audio/out` and `audio_player_node`
plays it; the AEC node uses the player's `/audio/played` output as its
reference. This mirrors the chatbot wiring: the AEC models the speaker and room,
not the player's resampling and device buffering. Speech recognition or any
other consumer should subscribe to `audio_clean`. `audio_clean` uses
`SensorDataQoS`; `ros2 topic echo` defaults to reliable QoS, so pass
`--qos-profile sensor_data` (as above) or the subscription may not match.

## Tuning

- **`reference_lead_ms`** selects reference audio slightly after the
  microphone timestamp and adds the same amount to the delay hint given to
  WebRTC. The default of `0` is suitable when the reference is already buffered
  ahead of the microphone. A small lead such as `50` keeps the delay away from
  zero, where WebRTC's estimator has coarse resolution, and gives the reference
  transport some jitter room. The chatbot uses `50`.
- **`reference_wait_ms`** is how long a microphone frame may wait for its
  timestamped reference before the missing reference is treated as silence.
  The default `100` absorbs normal player and ROS jitter. Lower it to reduce
  output latency when the reference is reliable; raise it on slow systems.
- **`mic_buffer_ms`** is an additional fixed delay before microphone frames are
  processed. Timestamp alignment already guarantees that the reference for a
  frame is selected correctly, so this is now a safety/jitter buffer rather
  than the mechanism that aligns the streams. Larger values add output latency;
  the maximum is 500 ms.
- **`reference_timeout_ms`** controls what happens after playback stops. After
  this long without a reference frame, the node feeds silence to the AEC and
  resets it once, so an old echo path does not leak into the next playback. A
  lower value resets sooner; too low a value can reset during normal gaps
  between reference messages.
- **`output_chunk_ms`** trades latency for message rate. Smaller chunks publish
  more often and add less latency.
- **`stream_delay_ms`** is only an initial hint for WebRTC, which refines the
  delay itself. The effective hint is `stream_delay_ms + reference_lead_ms`.
  Increase it if the speaker has an unusually large device buffer.
- **`enable_noise_suppression`** is off by default so the output stays close to
  the microphone signal apart from echo cancellation. Enable it when a
  downstream VAD must not fire on the room-noise floor; the chatbot enables it
  for exactly that reason. **`enable_agc`** remains off unless the downstream
  pipeline needs level normalization.

The microphone and reference FIFOs are capped at one second. If a publisher runs
far ahead of real time, the oldest audio is dropped, the processor is reset, and
a throttled warning is logged instead of allowing unbounded latency.

## Limitations

- **The reference is the player's output, not the acoustic path.** The AEC
  still has to model the speaker, room, and microphone. Nonlinear speaker
  distortion and room reflections can leave residual echo; noise suppression
  on the output reduces the remaining floor.
- **Separate devices drift.** A microphone and a speaker on different USB
  devices have independent clocks. Timestamp alignment removes ROS transport
  jitter, but long sessions may still need a reset or a larger
  `reference_wait_ms`/`mic_buffer_ms`.
- **16 kHz processing bandwidth at the default rate.** At 16 kHz, cancellation
  is limited to the speech band. This is the right trade-off for speech
  recognition but not for wideband music; raise `rate` to process a wider band.
- **Reconvergence after playback starts.** The first fraction of a second after
  playback resumes may contain residual echo while the AEC adapts again. A
  small `reference_lead_ms` and a retained filter across turns reduce this
  transient.

## Reference requirement

The reference topic must carry the far-end signal (what the speaker plays),
not an unrelated audio stream. The AEC cannot cancel sound whose source it
never sees.

The best reference is the `played` topic published by `audio_player_node`,
because it contains the exact post-resampling, post-channel-conversion frames
written to the output device. Remap `audio_reference` to that topic (for
example `/audio/played`) for the best cancellation. The older wiring, where
both the playback source (`tts_node`, `music_node`, or another
`AudioStamped` publisher) and `audio_player_node` use `audio_reference`, also
works, but the player's resampling, buffering, and device latency then become
part of the echo path the AEC has to model.

Playback that bypasses ROS, such as audio played directly through ALSA,
PulseAudio, or PipeWire outside `audio_common`, provides no reference and
cannot be cancelled by this node.
