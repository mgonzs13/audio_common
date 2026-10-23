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

#include <stdexcept>
#include <vector>

#include "audio_aec/webrtc_aec.hpp"

TEST(WebrtcAecTest, ProcessesSilenceAndResets) {
  audio_aec::WebrtcAec aec(16000, true, false, false);
  ASSERT_EQ(aec.sample_rate(), 16000);
  ASSERT_EQ(aec.frame_size(), 160);

  std::vector<float> input(160, 0.0f);
  std::vector<float> output(160, 1.0f);
  EXPECT_TRUE(aec.process_render(input.data(), input.size()));
  EXPECT_TRUE(aec.process_capture(input.data(), output.data(), output.size()));
  for (float sample : output) {
    EXPECT_NEAR(sample, 0.0f, 1e-6f);
  }

  aec.set_delay_ms(20);
  EXPECT_TRUE(aec.process_capture(input.data(), output.data(), output.size()));
  aec.reset();
  EXPECT_TRUE(aec.process_render(input.data(), input.size()));
  EXPECT_TRUE(aec.process_capture(input.data(), output.data(), output.size()));
}

TEST(WebrtcAecTest, RejectsWrongFrameSize) {
  audio_aec::WebrtcAec aec(16000, true, false, false);
  std::vector<float> input(100, 0.0f);
  std::vector<float> output(100, 0.0f);
  EXPECT_FALSE(aec.process_render(input.data(), input.size()));
  EXPECT_FALSE(aec.process_capture(input.data(), output.data(), output.size()));
}

TEST(WebrtcAecTest, RejectsUnsupportedSampleRate) {
  EXPECT_THROW(audio_aec::WebrtcAec(11025, true, false, false),
               std::invalid_argument);
  EXPECT_THROW(audio_aec::WebrtcAec(0, true, false, false),
               std::invalid_argument);
  EXPECT_THROW(audio_aec::WebrtcAec(-16000, true, false, false),
               std::invalid_argument);
}

TEST(WebrtcAecTest, LastErrorTracksProcessing) {
  audio_aec::WebrtcAec aec(16000, true, false, false);
  std::vector<float> input(160, 0.0f);
  std::vector<float> output(160, 0.0f);

  EXPECT_EQ(aec.last_error(), 0);
  EXPECT_TRUE(aec.process_render(input.data(), input.size()));
  EXPECT_EQ(aec.last_error(), 0);
  EXPECT_TRUE(aec.process_capture(input.data(), output.data(), output.size()));
  EXPECT_EQ(aec.last_error(), 0);

  EXPECT_FALSE(aec.process_render(input.data(), 100));
  EXPECT_EQ(aec.last_error(), 0);

  aec.reset();
  EXPECT_EQ(aec.last_error(), 0);
}
