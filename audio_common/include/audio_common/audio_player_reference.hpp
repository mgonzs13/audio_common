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

#ifndef AUDIO_COMMON__AUDIO_PLAYER_REFERENCE_HPP_
#define AUDIO_COMMON__AUDIO_PLAYER_REFERENCE_HPP_

#include <cstddef>

#include <builtin_interfaces/msg/time.hpp>

#include "audio_common_msgs/msg/audio_stamped.hpp"

namespace audio_common {

/**
 * @brief Build the playback-reference message from the frames being written
 *        to the output device.
 *
 * The message contains mono or interleaved float32 samples in the output
 * format, so an AEC node can use exactly what the player sends to the device
 * as its far-end reference. The header stamp is supplied by the caller and
 * can describe the intended render time.
 *
 * @param data Interleaved output samples.
 * @param frames Number of audio frames in @p data.
 * @param channels Number of output channels.
 * @param rate Output sample rate in Hz.
 * @param stamp Header timestamp for the message.
 * @return AudioStamped message carrying the played frames.
 */
audio_common_msgs::msg::AudioStamped
make_reference_message(const float *data, std::size_t frames, int channels,
                       int rate, const builtin_interfaces::msg::Time &stamp);

} // namespace audio_common

#endif // AUDIO_COMMON__AUDIO_PLAYER_REFERENCE_HPP_
