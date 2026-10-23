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

#include "audio_aec/audio_converter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace audio_aec {
namespace {

float sanitize_sample(float value) {
  if (!std::isfinite(value)) {
    return 0.0f;
  }
  return std::min(1.0f, std::max(-1.0f, value));
}

template <typename T> float to_normalized_signed(T value) {
  const double scale = static_cast<double>(std::numeric_limits<T>::max()) + 1.0;
  return static_cast<float>(static_cast<double>(value) / scale);
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

std::vector<float> downmix(const std::vector<float> &interleaved,
                           int channels) {
  const std::size_t channel_count = static_cast<std::size_t>(channels);
  if (interleaved.size() % channel_count != 0) {
    throw std::invalid_argument(
        "Audio data length is not a multiple of the channel count");
  }

  const std::size_t frame_count = interleaved.size() / channel_count;
  std::vector<float> mono(frame_count);

  for (std::size_t frame = 0; frame < frame_count; ++frame) {
    double sum = 0.0;
    for (std::size_t channel = 0; channel < channel_count; ++channel) {
      sum += interleaved[frame * channel_count + channel];
    }
    mono[frame] = sanitize_sample(
        static_cast<float>(sum / static_cast<double>(channel_count)));
  }

  return mono;
}

} // namespace

std::vector<float>
AudioConverter::to_mono_float(const audio_common_msgs::msg::AudioData &data,
                              int format, int channels) {
  if (channels <= 0) {
    throw std::invalid_argument("Channel count must be positive");
  }

  std::vector<float> interleaved;

  switch (format) {
  case kPaFloat32:
    interleaved.reserve(data.float32_data.size());
    for (const float value : data.float32_data) {
      interleaved.push_back(sanitize_sample(value));
    }
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

  return downmix(interleaved, channels);
}

void AudioConverter::from_mono_float(const std::vector<float> &samples,
                                     int format,
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

} // namespace audio_aec
