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

#ifndef AUDIO_COMMON_DSP__TIME_UTILS_HPP_
#define AUDIO_COMMON_DSP__TIME_UTILS_HPP_

#include <cstdint>

namespace audio_common_dsp {

constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

/// @brief Combine seconds and nanoseconds into a signed nanosecond count.
inline int64_t to_nanoseconds(std::int32_t seconds, std::uint32_t nanoseconds) {
  return static_cast<int64_t>(seconds) * kNanosecondsPerSecond +
         static_cast<int64_t>(nanoseconds);
}

/// @brief Split a signed nanosecond count into seconds and nanoseconds.
inline void from_nanoseconds(std::int64_t nanoseconds, std::int32_t &seconds,
                             std::uint32_t &nanos) {
  if (nanoseconds < 0) {
    nanoseconds = 0;
  }
  seconds = static_cast<std::int32_t>(nanoseconds / kNanosecondsPerSecond);
  nanos = static_cast<std::uint32_t>(nanoseconds % kNanosecondsPerSecond);
}

} // namespace audio_common_dsp

#endif // AUDIO_COMMON_DSP__TIME_UTILS_HPP_
