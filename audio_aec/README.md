# audio_aec

## Overview

`audio_aec` provides `aec_node`, an acoustic echo cancellation (AEC) node for
ROS 2. The node removes the sound that the robot's own speaker is playing from
the microphone signal, so the microphone can keep hearing the user during
playback. It subscribes to a microphone topic and to a reference topic that
carries the audio message the player is asked to play, and publishes cleaned mono
audio.

The motivating use case is text-to-speech: `tts_node` generates speech that
`audio_player_node` plays through the speaker, and `audio_capturer_node` picks
that speech up again through the microphone. By feeding the same TTS audio to
`aec_node` as a reference, the echo is cancelled and downstream speech
recognition receives mostly the user's voice. The same mechanism works for
`music_node` or any other `AudioStamped` source that feeds the speaker.

Echo cancellation is performed by WebRTC AudioProcessing. The node resamples
both inputs to a fixed internal rate, processes 10 ms frames, and publishes
mono output in chunks. No existing `audio_common` node needs to be modified;
the graph is wired with topic remapping.

## Install

```shell
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to audio_aec
source install/setup.bash
```

Ubuntu 20.04 through 24.04 ship WebRTC AudioProcessing 0.3.1, while Ubuntu 25.10
and later ship 1.3. The package supports both APIs and selects the right one at
build time.

## Topics

| Direction | Default topic     | Type                                 | QoS             |
| --------- | ----------------- | ------------------------------------ | --------------- |
| Subscribe | `audio`           | `audio_common_msgs/msg/AudioStamped` | `SensorDataQoS` |
| Subscribe | `audio_reference` | `audio_common_msgs/msg/AudioStamped` | `SensorDataQoS` |
| Publish   | `audio_clean`     | `audio_common_msgs/msg/AudioStamped` | `SensorDataQoS` |

The topic names are parameters, so the graph can be adapted without renaming
anything.

Both inputs accept all five PortAudio sample formats supported by
`audio_common_msgs` (`paFloat32`, `paInt32`, `paInt16`, `paInt8`, and `paUInt8`)
at any positive sample rate and channel count. Multi-channel input is downmixed
to mono by averaging the channels. The output is always mono, uses the
configured `format` and `rate`, and carries the microphone input's `frame_id`.
Its stamp is the capture stamp of the first sample in the published chunk, so
the output stays aligned with the original capture timeline even though the node
adds `mic_buffer_ms` of latency.

## Parameters

| Name                       | Type   | Default           | Meaning                                                                                    |
| -------------------------- | ------ | ----------------- | ------------------------------------------------------------------------------------------ |
| `mic_topic`                | string | `audio`           | Microphone input topic                                                                     |
| `reference_topic`          | string | `audio_reference` | Playback reference topic                                                                   |
| `output_topic`             | string | `audio_clean`     | Cleaned output topic                                                                       |
| `rate`                     | int    | `16000`           | Internal processing and output rate; one of 8000, 16000, 32000, 48000                      |
| `format`                   | int    | `8` (`paInt16`)   | Output PortAudio sample format constant                                                    |
| `mic_buffer_ms`            | int    | `100`             | Fixed microphone delay used to make sure reference frames are available first; at most 500 |
| `output_chunk_ms`          | int    | `20`              | Duration of each published output message                                                  |
| `stream_delay_ms`          | int    | `0`               | Initial delay hint passed to WebRTC; the library refines it                                |
| `enable_high_pass`         | bool   | `true`            | Enable WebRTC's high-pass filter                                                           |
| `enable_noise_suppression` | bool   | `false`           | Enable WebRTC noise suppression after AEC                                                  |
| `enable_agc`               | bool   | `false`           | Enable WebRTC automatic gain control after AEC                                             |
| `reference_timeout_ms`     | int    | `500`             | After this long without a reference frame, feed silence and reset AEC once                 |

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
ros2 run audio_common tts_node --ros-args -r audio:=audio_reference
```

### 3. Playback

```shell
ros2 run audio_common audio_player_node --ros-args -r audio:=audio_reference
```

### 4. AEC node

```shell
ros2 run audio_aec aec_node
```

The defaults are microphone `/audio`, reference `/audio_reference`, and output
`/audio_clean`.

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
  -p device:=<output_device_index>
```

`audio_capturer_node` keeps publishing on `audio`, which is `aec_node`'s default
microphone topic. Both `tts_node` and `audio_player_node` are remapped to
`audio_reference`, so the AEC node sees the audio message the player is asked to
play. Speech recognition or any other consumer should subscribe to `audio_clean`.
`audio_clean` uses `SensorDataQoS`; `ros2 topic echo` defaults to reliable QoS, so
pass `--qos-profile sensor_data` (as above) or the subscription may not match.

## Tuning

- **`mic_buffer_ms`** holds the microphone path back so the corresponding
  reference audio arrives before the microphone echo is processed. It must be at
  least the largest expected gap between reference messages. `tts_node`
  publishes 4096-frame chunks, which is about 186 ms at 22.05 kHz, so the default
  of 100 ms may be too small on slow systems; increase it (for example to 200)
  if echo remains at the start of each TTS burst. Larger values add output
  latency; the maximum is 500 ms.
- **`reference_timeout_ms`** controls what happens after playback stops. After
  this long without a reference frame, the node feeds silence to the AEC and
  resets it once, so an old echo path does not leak into the next playback. A
  lower value resets sooner; too low a value can reset during normal gaps
  between reference messages.
- **`output_chunk_ms`** trades latency for message rate. Smaller chunks publish
  more often and add less latency.
- **`stream_delay_ms`** is only an initial hint for WebRTC, which refines the
  delay itself. Increase it if the speaker has an unusually large device buffer.
- **`enable_noise_suppression`** and **`enable_agc`** are off by default so the
  output stays close to the microphone signal apart from echo cancellation.
  Enable them if the downstream pipeline needs them.

The microphone and reference FIFOs are capped at one second. If a publisher runs
far ahead of real time, the oldest audio is dropped, the processor is reset, and
a throttled warning is logged instead of allowing unbounded latency.

## Limitations

- **The reference is captured before the player.** The player's resampling,
  channel conversion, and device buffering are not part of the reference. AEC
  adapts to the combined linear path, but the resampling mismatch can leave
  residual high-frequency echo. This is acceptable for speech and is the cost of
  not modifying `audio_player_node`.
- **Separate devices drift.** A microphone and a speaker on different USB
  devices have independent clocks. WebRTC tracks small delay changes, but long
  sessions may need a reset or a larger `mic_buffer_ms`.
- **16 kHz processing bandwidth at the default rate.** At 16 kHz, cancellation
  is limited to the speech band. This is the right trade-off for speech
  recognition but not for wideband music; raise `rate` to process a wider band.
- **Reconvergence after playback starts.** The first fraction of a second after
  playback resumes may contain residual echo while the AEC adapts again.

## Reference requirement

The reference topic must carry the audio message the player is asked to play,
not an unrelated audio stream. The AEC needs the far-end signal (what the
speaker plays) to model the echo path; it cannot cancel sound whose source it
never sees.

In practice, remap both the playback source (`tts_node`, `music_node`, or
another `AudioStamped` publisher) and `audio_player_node` to the same topic, by
default `audio_reference`. Playback that bypasses ROS, such as audio played
directly through ALSA, PulseAudio, or PipeWire outside `audio_common`, provides
no reference and cannot be cancelled by this node.
