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

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

#include "audio_common_dsp/multi_channel_resampler.hpp"
#include "audio_common_dsp/resampler.hpp"

namespace {

std::vector<float> sine(double frequency, int rate, double seconds,
                        float amplitude) {
  const size_t frames = static_cast<size_t>(rate * seconds);
  std::vector<float> data(frames);
  for (size_t i = 0; i < frames; ++i) {
    data[i] = amplitude *
              static_cast<float>(std::sin(2.0 * M_PI * frequency * i / rate));
  }
  return data;
}

double rms(const std::vector<float> &data) {
  double sum = 0.0;
  for (float sample : data) {
    sum += static_cast<double>(sample) * sample;
  }
  return data.empty() ? 0.0 : std::sqrt(sum / data.size());
}

double estimate_frequency(const std::vector<float> &data, int rate) {
  int crossings = 0;
  for (size_t i = 1; i < data.size(); ++i) {
    if ((data[i - 1] < 0.0f && data[i] >= 0.0f) ||
        (data[i - 1] >= 0.0f && data[i] < 0.0f)) {
      ++crossings;
    }
  }
  return crossings / 2.0 * rate / static_cast<double>(data.size());
}

} // namespace

TEST(ResamplerTest, Downsample48kTo16kPreservesTone) {
  const auto input = sine(1000.0, 48000, 0.5, 0.5);
  audio_common_dsp::Resampler resampler;
  const auto output = resampler.process(input, 48000, 16000);
  ASSERT_NEAR(output.size(), input.size() / 3, 2u);
  EXPECT_NEAR(rms(output), 0.5 / std::sqrt(2.0), 0.05);
  EXPECT_NEAR(estimate_frequency(output, 16000), 1000.0, 20.0);
}

TEST(ResamplerTest, 22050To16000ProducesExpectedLength) {
  const auto input = sine(440.0, 22050, 1.0, 0.4);
  audio_common_dsp::Resampler resampler;
  const auto output = resampler.process(input, 22050, 16000);
  EXPECT_NEAR(output.size(), 16000u, 2u);
  EXPECT_NEAR(estimate_frequency(output, 16000), 440.0, 10.0);
}

TEST(ResamplerTest, StreamingMatchesOneShot) {
  const auto input = sine(440.0, 48000, 1.0, 0.4);

  audio_common_dsp::Resampler one_shot;
  const auto expected = one_shot.process(input, 48000, 16000);

  audio_common_dsp::Resampler streaming;
  std::vector<float> actual;
  for (size_t offset = 0; offset < input.size(); offset += 512) {
    const size_t count = std::min<size_t>(512, input.size() - offset);
    const std::vector<float> block(input.begin() + offset,
                                   input.begin() + offset + count);
    const auto converted = streaming.process(block, 48000, 16000);
    actual.insert(actual.end(), converted.begin(), converted.end());
  }

  ASSERT_EQ(actual.size(), expected.size());
  for (size_t i = 0; i < actual.size(); ++i) {
    EXPECT_NEAR(actual[i], expected[i], 1e-4f);
  }
}

TEST(ResamplerTest, ResetClearsFilterAndPhase) {
  const auto input = sine(1000.0, 48000, 0.25, 0.5);
  audio_common_dsp::Resampler resampler;
  resampler.process(input, 48000, 16000);
  resampler.reset();

  audio_common_dsp::Resampler fresh;
  const auto after_reset = resampler.process(input, 48000, 16000);
  const auto expected = fresh.process(input, 48000, 16000);
  ASSERT_EQ(after_reset.size(), expected.size());
  for (size_t i = 0; i < after_reset.size(); ++i) {
    EXPECT_NEAR(after_reset[i], expected[i], 1e-5f);
  }
}

TEST(ResamplerTest, AttenuatesToneAboveOutputNyquist) {
  const auto input = sine(10000.0, 48000, 0.5, 0.5);
  audio_common_dsp::Resampler resampler;
  const auto output = resampler.process(input, 48000, 16000);

  ASSERT_NEAR(output.size(), input.size() / 3, 2u);
  // Without the anti-alias filter, 10 kHz would alias to 6 kHz at full
  // amplitude (RMS about 0.35). The 4th-order filter must attenuate it well.
  EXPECT_LT(rms(output), 0.15);
}

TEST(ResamplerTest, RejectsNonPositiveRates) {
  audio_common_dsp::Resampler resampler;
  EXPECT_THROW(resampler.process({0.1f}, 0, 16000), std::invalid_argument);
  EXPECT_THROW(resampler.process({0.1f}, 48000, 0), std::invalid_argument);
  EXPECT_THROW(resampler.process({0.1f}, -1, 16000), std::invalid_argument);
  EXPECT_THROW(resampler.process({0.1f}, 16000, -1), std::invalid_argument);
}

TEST(ResamplerTest, FlushReturnsOutputRateSamplesForDownsampling) {
  auto input = sine(1000.0, 48000, 0.5, 0.5);
  input.push_back(0.25f);

  audio_common_dsp::Resampler resampler;
  auto output = resampler.process(input, 48000, 16000);
  const auto tail = resampler.flush();
  output.insert(output.end(), tail.begin(), tail.end());

  // 24001 input frames leave one sample buffered; it must be drained as one
  // 16 kHz output sample, not appended as a raw 48 kHz sample.
  EXPECT_LE(tail.size(), 1u);
  EXPECT_NEAR(output.size(), input.size() / 3.0, 1.0);
  EXPECT_NEAR(rms(output), 0.5 / std::sqrt(2.0), 0.05);
  EXPECT_NEAR(estimate_frequency(output, 16000), 1000.0, 20.0);
}

TEST(ResamplerTest, FlushReturnsAllOutputRateSamplesForUpsampling) {
  const auto input = sine(1000.0, 16000, 1.0, 0.5);

  audio_common_dsp::Resampler resampler;
  auto output = resampler.process(input, 16000, 48000);
  const auto tail = resampler.flush();
  output.insert(output.end(), tail.begin(), tail.end());

  // One buffered 16 kHz sample represents three 48 kHz output samples.
  EXPECT_EQ(tail.size(), 3u);
  EXPECT_EQ(output.size(), input.size() * 3);
  EXPECT_NEAR(rms(output), 0.5 / std::sqrt(2.0), 0.05);
  EXPECT_NEAR(estimate_frequency(output, 48000), 1000.0, 20.0);
}

TEST(MultiChannelResamplerTest, DownsampleStereoPreservesChannels) {
  const auto left = sine(1000.0, 48000, 0.5, 0.5);
  const auto right = sine(2000.0, 48000, 0.5, 0.5);
  std::vector<float> input(left.size() * 2);
  for (size_t i = 0; i < left.size(); ++i) {
    input[2 * i] = left[i];
    input[2 * i + 1] = right[i];
  }

  audio_common_dsp::MultiChannelResampler resampler;
  auto output = resampler.process(input, 2, 48000, 16000);
  const auto tail = resampler.flush();
  output.insert(output.end(), tail.begin(), tail.end());

  ASSERT_EQ(output.size() % 2, 0u);
  const size_t frames = output.size() / 2;
  EXPECT_NEAR(frames, left.size() / 3, 2u);

  std::vector<float> left_out(frames);
  std::vector<float> right_out(frames);
  for (size_t i = 0; i < frames; ++i) {
    left_out[i] = output[2 * i];
    right_out[i] = output[2 * i + 1];
  }
  EXPECT_NEAR(rms(left_out), 0.5 / std::sqrt(2.0), 0.05);
  EXPECT_NEAR(rms(right_out), 0.5 / std::sqrt(2.0), 0.05);
  EXPECT_NEAR(estimate_frequency(left_out, 16000), 1000.0, 20.0);
  EXPECT_NEAR(estimate_frequency(right_out, 16000), 2000.0, 40.0);
}

TEST(MultiChannelResamplerTest, SameRatePassesThrough) {
  audio_common_dsp::MultiChannelResampler resampler;
  const std::vector<float> input = {0.0f, 10.0f, 2.0f, 12.0f};
  EXPECT_EQ(resampler.process(input, 2, 48000, 48000), input);
  EXPECT_TRUE(resampler.flush().empty());
}

TEST(MultiChannelResamplerTest, FlushReturnsInterleavedTail) {
  audio_common_dsp::MultiChannelResampler resampler;
  EXPECT_TRUE(resampler.process({1.0f}, 1, 16000, 48000).empty());
  const auto tail = resampler.flush();
  EXPECT_EQ(tail.size(), 3u);
  for (float sample : tail) {
    EXPECT_FLOAT_EQ(sample, 1.0f);
  }
}

TEST(MultiChannelResamplerTest, RejectsInvalidChannels) {
  audio_common_dsp::MultiChannelResampler resampler;
  EXPECT_THROW(resampler.process({0.1f}, 0, 48000, 16000),
               std::invalid_argument);
  EXPECT_THROW(resampler.process({0.1f, 0.2f, 0.3f}, 2, 48000, 16000),
               std::invalid_argument);
  EXPECT_THROW(resampler.process({0.1f}, -1, 48000, 16000),
               std::invalid_argument);
}
