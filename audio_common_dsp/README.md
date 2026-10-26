# audio_common_dsp

Shared, ROS-free audio DSP utilities for the `audio_common` repository.

## Components

- `audio_common_dsp::AudioConverter` — converts the five PortAudio sample
  formats carried by `audio_common_msgs/msg/AudioData` to interleaved float
  samples, optionally downmixes to mono, sanitizes non-finite samples, and
  converts float samples back to the requested format.
- `audio_common_dsp::Resampler` — streaming mono sample-rate converter with a
  fourth-order Butterworth anti-alias filter for downsampling, linear
  interpolation, persistent phase/filter state, and an output-rate `flush()`.
- `audio_common_dsp::MultiChannelResampler` — interleaved multichannel wrapper
  around one `Resampler` per channel.
- `audio_common_dsp/sample_format.hpp` — shared PortAudio format constants,
  format validation, `int16_to_float`, and sample sanitization helpers.
- `audio_common_dsp/time_utils.hpp` — shared seconds/nanoseconds conversion.

## Use

```cpp
#include "audio_common_dsp/audio_converter.hpp"
#include "audio_common_dsp/resampler.hpp"

auto mono = audio_common_dsp::AudioConverter::to_mono_float(
    msg.audio.audio_data, msg.audio.info.format, msg.audio.info.channels);

audio_common_dsp::Resampler resampler;
auto resampled = resampler.process(mono, msg.audio.info.rate, 16000);
```

The package is currently used by `audio_aec`. It is intentionally separate from
the `audio_common` node package so DSP-only consumers do not pull in PortAudio
or espeak.

## Build

```shell
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to audio_common_dsp
colcon test --packages-select audio_common_dsp
```
