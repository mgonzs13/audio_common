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

#include <gtest/gtest.h>

#include <vector>

#include <portaudio.h>

#include "audio_common/audio_player_reference.hpp"

TEST(AudioPlayerReference, BuildsFloat32MessageFromPlayedFrames) {
  const std::vector<float> samples{0.1f, -0.2f, 0.3f, -0.4f};

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 34;

  const auto msg =
      audio_common::make_reference_message(samples.data(), 2, 2, 48000, stamp);

  EXPECT_EQ(msg.header.stamp.sec, 12);
  EXPECT_EQ(msg.header.stamp.nanosec, 34);
  EXPECT_EQ(msg.audio.info.format, paFloat32);
  EXPECT_EQ(msg.audio.info.channels, 2);
  EXPECT_EQ(msg.audio.info.rate, 48000);
  EXPECT_EQ(msg.audio.info.chunk, 2);
  EXPECT_EQ(msg.audio.audio_data.float32_data, samples);
}
