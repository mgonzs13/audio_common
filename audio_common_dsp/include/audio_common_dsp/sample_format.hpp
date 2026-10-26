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

#ifndef AUDIO_COMMON_DSP__SAMPLE_FORMAT_HPP_
#define AUDIO_COMMON_DSP__SAMPLE_FORMAT_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace audio_common_dsp {

/// @brief PortAudio sample format constants supported by the DSP utilities.
constexpr int kPaFloat32 = 1;
constexpr int kPaInt32 = 2;
constexpr int kPaInt16 = 8;
constexpr int kPaInt8 = 16;
constexpr int kPaUInt8 = 32;

/// @brief True if @p format is one of the supported PortAudio constants.
inline bool is_supported_format(int format) {
  return format == kPaFloat32 || format == kPaInt32 || format == kPaInt16 ||
         format == kPaInt8 || format == kPaUInt8;
}

/// @brief Convert a signed 16-bit PCM sample to a normalized float.
inline float int16_to_float(std::int16_t sample) {
  return static_cast<float>(sample) / 32768.0f;
}

/// @brief Replace non-finite samples with zero and clamp to [-1, 1].
inline float sanitize_sample(float sample) {
  if (!std::isfinite(sample)) {
    return 0.0f;
  }
  return std::min(1.0f, std::max(-1.0f, sample));
}

/// @brief Sanitize every sample in @p samples in place.
inline void sanitize_samples(std::vector<float> &samples) {
  for (float &sample : samples) {
    sample = sanitize_sample(sample);
  }
}

} // namespace audio_common_dsp

#endif // AUDIO_COMMON_DSP__SAMPLE_FORMAT_HPP_
