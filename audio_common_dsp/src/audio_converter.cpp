// MIT License
//
// Copyright (c) 2024 Miguel Ángel González Santamarta
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "audio_common_dsp/audio_converter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace audio_common_dsp {
namespace {

template <typename T> float to_normalized_signed(T value) {
  const double scale = static_cast<double>(std::numeric_limits<T>::max()) + 1.0;
  return static_cast<float>(static_cast<double>(value) / scale);
}

template <> float to_normalized_signed<std::int16_t>(std::int16_t value) {
  return int16_to_float(value);
}

float to_normalized_uint8(std::uint8_t value) {
  return (static_cast<float>(value) - 128.0f) / 128.0f;
}

template <typename T> T from_normalized_signed(float value) {
  const float clamped = sanitize_sample(value);
  const double scale =
      clamped >= 0.0f ? static_cast<double>(std::numeric_limits<T>::max())
                      : -static_cast<double>(std::numeric_limits<T>::min());
  return static_cast<T>(std::lround(static_cast<double>(clamped) * scale));
}

std::uint8_t from_normalized_uint8(float value) {
  const float clamped = sanitize_sample(value);
  const float scaled = clamped * 128.0f + 128.0f;
  return static_cast<std::uint8_t>(
      std::lround(std::min(255.0f, std::max(0.0f, scaled))));
}

} // namespace

bool AudioConverter::is_supported_format(int format) {
  return audio_common_dsp::is_supported_format(format);
}

std::vector<float>
AudioConverter::to_float(const audio_common_msgs::msg::AudioData &data,
                         int format, int channels) {
  if (channels <= 0) {
    throw std::invalid_argument("Channel count must be positive");
  }

  std::vector<float> interleaved;

  switch (format) {
  case kPaFloat32:
    interleaved.assign(data.float32_data.begin(), data.float32_data.end());
    break;
  case kPaInt32:
    interleaved.reserve(data.int32_data.size());
    for (const auto value : data.int32_data) {
      interleaved.push_back(to_normalized_signed(value));
    }
    break;
  case kPaInt16:
    interleaved.reserve(data.int16_data.size());
    for (const auto value : data.int16_data) {
      interleaved.push_back(to_normalized_signed(value));
    }
    break;
  case kPaInt8:
    interleaved.reserve(data.int8_data.size());
    for (const auto value : data.int8_data) {
      interleaved.push_back(to_normalized_signed(value));
    }
    break;
  case kPaUInt8:
    interleaved.reserve(data.uint8_data.size());
    for (const auto value : data.uint8_data) {
      interleaved.push_back(to_normalized_uint8(value));
    }
    break;
  default:
    throw std::invalid_argument("Unsupported audio format: " +
                                std::to_string(format));
  }

  if (interleaved.size() % static_cast<std::size_t>(channels) != 0) {
    throw std::invalid_argument(
        "Audio data length is not a multiple of the channel count");
  }

  sanitize_samples(interleaved);
  return interleaved;
}

std::vector<float>
AudioConverter::to_mono_float(const audio_common_msgs::msg::AudioData &data,
                              int format, int channels) {
  const std::vector<float> interleaved =
      AudioConverter::to_float(data, format, channels);
  return AudioConverter::convert_channels(interleaved, channels, 1);
}

void AudioConverter::from_float(const std::vector<float> &samples, int format,
                                audio_common_msgs::msg::AudioData &out) {
  switch (format) {
  case kPaFloat32:
    out.float32_data.resize(samples.size());
    std::transform(samples.begin(), samples.end(), out.float32_data.begin(),
                   [](float value) { return sanitize_sample(value); });
    break;
  case kPaInt32:
    out.int32_data.resize(samples.size());
    std::transform(
        samples.begin(), samples.end(), out.int32_data.begin(),
        [](float value) { return from_normalized_signed<int32_t>(value); });
    break;
  case kPaInt16:
    out.int16_data.resize(samples.size());
    std::transform(
        samples.begin(), samples.end(), out.int16_data.begin(),
        [](float value) { return from_normalized_signed<int16_t>(value); });
    break;
  case kPaInt8:
    out.int8_data.resize(samples.size());
    std::transform(
        samples.begin(), samples.end(), out.int8_data.begin(),
        [](float value) { return from_normalized_signed<int8_t>(value); });
    break;
  case kPaUInt8:
    out.uint8_data.resize(samples.size());
    std::transform(samples.begin(), samples.end(), out.uint8_data.begin(),
                   [](float value) { return from_normalized_uint8(value); });
    break;
  default:
    throw std::invalid_argument("Unsupported audio format: " +
                                std::to_string(format));
  }
}

void AudioConverter::from_mono_float(const std::vector<float> &samples,
                                     int format,
                                     audio_common_msgs::msg::AudioData &out) {
  AudioConverter::from_float(samples, format, out);
}

std::vector<float>
AudioConverter::convert_channels(const std::vector<float> &samples,
                                 int input_channels, int output_channels) {
  if (input_channels <= 0 || output_channels <= 0) {
    throw std::invalid_argument("Channel counts must be positive");
  }

  if (samples.size() % static_cast<std::size_t>(input_channels) != 0) {
    throw std::invalid_argument(
        "Audio data length is not a multiple of the channel count");
  }

  if (input_channels == output_channels) {
    return samples;
  }

  const std::size_t frames =
      samples.size() / static_cast<std::size_t>(input_channels);

  if (input_channels == 1) {
    std::vector<float> output(frames *
                              static_cast<std::size_t>(output_channels));
    for (std::size_t frame = 0; frame < frames; ++frame) {
      for (int channel = 0; channel < output_channels; ++channel) {
        output[frame * static_cast<std::size_t>(output_channels) +
               static_cast<std::size_t>(channel)] = samples[frame];
      }
    }

    return output;
  }

  if (output_channels == 1) {
    std::vector<float> output(frames);

    for (std::size_t frame = 0; frame < frames; ++frame) {
      double sum = 0.0;

      for (int channel = 0; channel < input_channels; ++channel) {
        sum += samples[frame * static_cast<std::size_t>(input_channels) +
                       static_cast<std::size_t>(channel)];
      }

      output[frame] = sanitize_sample(
          static_cast<float>(sum / static_cast<double>(input_channels)));
    }

    return output;
  }

  throw std::invalid_argument("Unsupported channel conversion from " +
                              std::to_string(input_channels) + " to " +
                              std::to_string(output_channels));
}

} // namespace audio_common_dsp
