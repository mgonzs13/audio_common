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

#include "audio_common/audio_player_reference.hpp"

#include <cstddef>

#include <portaudio.h>

namespace audio_common {

audio_common_msgs::msg::AudioStamped
make_reference_message(const float *data, std::size_t frames, int channels,
                       int rate, const builtin_interfaces::msg::Time &stamp) {
  audio_common_msgs::msg::AudioStamped msg;
  msg.header.stamp = stamp;
  msg.audio.info.format = paFloat32;
  msg.audio.info.channels = channels;
  msg.audio.info.rate = rate;
  msg.audio.info.chunk = static_cast<int32_t>(frames);

  if (data != nullptr && frames > 0 && channels > 0) {
    msg.audio.audio_data.float32_data.assign(data, data + frames * channels);
  }

  return msg;
}

} // namespace audio_common
