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

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "audio_common_dsp/audio_converter.hpp"
#include "audio_common_msgs/msg/audio_data.hpp"

using audio_common_dsp::AudioConverter;
using audio_common_msgs::msg::AudioData;

TEST(AudioConverterTest, Float32RoundTrip) {
  AudioData data;
  data.float32_data = {0.0f, 0.5f, -0.5f, 1.0f, -1.0f};
  const auto mono =
      AudioConverter::to_mono_float(data, audio_common_dsp::kPaFloat32, 1);
  ASSERT_EQ(mono.size(), 5u);
  EXPECT_FLOAT_EQ(mono[1], 0.5f);

  AudioData out;
  AudioConverter::from_mono_float(mono, audio_common_dsp::kPaFloat32, out);
  EXPECT_EQ(out.float32_data, data.float32_data);
}

TEST(AudioConverterTest, Int16ClipsAndScales) {
  AudioData data;
  data.int16_data = {0, 16384, -16384, 32767, -32768};
  const auto mono =
      AudioConverter::to_mono_float(data, audio_common_dsp::kPaInt16, 1);
  ASSERT_EQ(mono.size(), 5u);
  EXPECT_NEAR(mono[1], 0.5f, 1e-4f);
  EXPECT_NEAR(mono[2], -0.5f, 1e-4f);

  AudioData out;
  AudioConverter::from_mono_float({2.0f, -2.0f}, audio_common_dsp::kPaInt16,
                                  out);
  EXPECT_EQ(out.int16_data[0], std::numeric_limits<int16_t>::max());
  EXPECT_EQ(out.int16_data[1], std::numeric_limits<int16_t>::min());
}

TEST(AudioConverterTest, Int32RoundTrip) {
  const std::vector<int32_t> values = {0, 1 << 30, -(1 << 30), 1 << 20,
                                       -(1 << 20)};
  AudioData data;
  data.int32_data = values;
  const auto mono =
      AudioConverter::to_mono_float(data, audio_common_dsp::kPaInt32, 1);
  ASSERT_EQ(mono.size(), values.size());

  AudioData out;
  AudioConverter::from_mono_float(mono, audio_common_dsp::kPaInt32, out);
  EXPECT_EQ(out.int32_data, values);
}

TEST(AudioConverterTest, Int8RoundTrip) {
  const std::vector<int8_t> values = {0, 32, -32, 64, -64};
  AudioData data;
  data.int8_data = values;
  const auto mono =
      AudioConverter::to_mono_float(data, audio_common_dsp::kPaInt8, 1);
  ASSERT_EQ(mono.size(), values.size());
  EXPECT_NEAR(mono[1], 0.25f, 1e-6f);

  AudioData out;
  AudioConverter::from_mono_float(mono, audio_common_dsp::kPaInt8, out);
  EXPECT_EQ(out.int8_data, values);
}

TEST(AudioConverterTest, UInt8RoundTrip) {
  AudioData data;
  data.uint8_data = {0, 128, 255};
  const auto mono =
      AudioConverter::to_mono_float(data, audio_common_dsp::kPaUInt8, 1);
  ASSERT_EQ(mono.size(), 3u);
  EXPECT_NEAR(mono[0], -1.0f, 1e-6f);
  EXPECT_NEAR(mono[1], 0.0f, 1e-6f);

  AudioData out;
  AudioConverter::from_mono_float(mono, audio_common_dsp::kPaUInt8, out);
  EXPECT_EQ(out.uint8_data, data.uint8_data);
}

TEST(AudioConverterTest, DownmixesStereoByAveraging) {
  AudioData data;
  data.float32_data = {0.2f, 0.4f, -0.2f, 0.0f};
  const auto mono =
      AudioConverter::to_mono_float(data, audio_common_dsp::kPaFloat32, 2);
  ASSERT_EQ(mono.size(), 2u);
  EXPECT_NEAR(mono[0], 0.3f, 1e-6f);
  EXPECT_NEAR(mono[1], -0.1f, 1e-6f);
}

TEST(AudioConverterTest, RejectsInvalidShapes) {
  AudioData data;
  data.float32_data = {0.1f, 0.2f, 0.3f};
  EXPECT_THROW(
      AudioConverter::to_mono_float(data, audio_common_dsp::kPaFloat32, 0),
      std::invalid_argument);
  EXPECT_THROW(
      AudioConverter::to_mono_float(data, audio_common_dsp::kPaFloat32, 2),
      std::invalid_argument);
  EXPECT_THROW(AudioConverter::to_mono_float(data, 999, 1),
               std::invalid_argument);
}

TEST(AudioConverterTest, ReadsOnlyMatchingArray) {
  AudioData data;
  data.float32_data = {0.9f};
  data.int32_data = {1 << 30};
  data.int16_data = {16384};
  data.int8_data = {64};
  data.uint8_data = {192};

  const auto mono =
      AudioConverter::to_mono_float(data, audio_common_dsp::kPaInt16, 1);
  ASSERT_EQ(mono.size(), 1u);
  EXPECT_NEAR(mono[0], 0.5f, 1e-4f);
}

TEST(AudioConverterTest, WritesOnlyMatchingArray) {
  AudioData out;
  out.float32_data = {0.9f};
  out.int32_data = {1};
  out.int16_data = {2};
  out.int8_data = {3};
  out.uint8_data = {4};

  AudioConverter::from_mono_float({0.5f}, audio_common_dsp::kPaInt16, out);

  ASSERT_EQ(out.int16_data.size(), 1u);
  EXPECT_EQ(out.int16_data[0], 16384);
  EXPECT_EQ(out.float32_data, std::vector<float>({0.9f}));
  EXPECT_EQ(out.int32_data, std::vector<int32_t>({1}));
  EXPECT_EQ(out.int8_data, std::vector<int8_t>({3}));
  EXPECT_EQ(out.uint8_data, std::vector<uint8_t>({4}));
}

TEST(AudioConverterTest, RejectsUnsupportedOutputFormat) {
  AudioData out;
  EXPECT_THROW(AudioConverter::from_mono_float({0.1f}, 999, out),
               std::invalid_argument);
}

TEST(AudioConverterTest, Float32SanitizesNonFiniteAndClamps) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();

  AudioData data;
  data.float32_data = {nan, inf, -inf, 2.0f, -2.0f, 0.25f};
  const auto mono =
      AudioConverter::to_mono_float(data, audio_common_dsp::kPaFloat32, 1);
  ASSERT_EQ(mono.size(), 6u);
  EXPECT_FLOAT_EQ(mono[0], 0.0f);
  EXPECT_FLOAT_EQ(mono[1], 0.0f);
  EXPECT_FLOAT_EQ(mono[2], 0.0f);
  EXPECT_FLOAT_EQ(mono[3], 1.0f);
  EXPECT_FLOAT_EQ(mono[4], -1.0f);
  EXPECT_FLOAT_EQ(mono[5], 0.25f);

  AudioData stereo;
  stereo.float32_data = {nan, 1.0f, inf, 0.0f};
  const auto downmixed =
      AudioConverter::to_mono_float(stereo, audio_common_dsp::kPaFloat32, 2);
  ASSERT_EQ(downmixed.size(), 2u);
  EXPECT_FLOAT_EQ(downmixed[0], 0.5f);
  EXPECT_FLOAT_EQ(downmixed[1], 0.0f);

  AudioData out;
  out.float32_data = {0.5f};
  AudioConverter::from_mono_float({nan, inf, -inf, 2.0f, -2.0f, 0.25f},
                                  audio_common_dsp::kPaFloat32, out);
  ASSERT_EQ(out.float32_data.size(), 6u);
  EXPECT_FLOAT_EQ(out.float32_data[0], 0.0f);
  EXPECT_FLOAT_EQ(out.float32_data[1], 0.0f);
  EXPECT_FLOAT_EQ(out.float32_data[2], 0.0f);
  EXPECT_FLOAT_EQ(out.float32_data[3], 1.0f);
  EXPECT_FLOAT_EQ(out.float32_data[4], -1.0f);
  EXPECT_FLOAT_EQ(out.float32_data[5], 0.25f);
}

TEST(AudioConverterTest, ToFloatPreservesChannels) {
  AudioData data;
  data.int16_data = {0, 16384, -16384, 32767};
  const auto samples =
      AudioConverter::to_float(data, audio_common_dsp::kPaInt16, 2);
  ASSERT_EQ(samples.size(), 4u);
  EXPECT_NEAR(samples[0], 0.0f, 1e-4f);
  EXPECT_NEAR(samples[1], 0.5f, 1e-4f);
  EXPECT_NEAR(samples[2], -0.5f, 1e-4f);
  EXPECT_NEAR(samples[3], 0.99997f, 1e-4f);
}

TEST(AudioConverterTest, FromFloatPreservesChannels) {
  AudioData out;
  AudioConverter::from_float({0.0f, 0.5f, -0.5f, 1.0f},
                             audio_common_dsp::kPaFloat32, out);
  EXPECT_EQ(out.float32_data, (std::vector<float>{0.0f, 0.5f, -0.5f, 1.0f}));
}

TEST(AudioConverterTest, ConvertChannelsMonoToStereo) {
  const auto stereo = AudioConverter::convert_channels({0.3f, -0.1f}, 1, 2);
  EXPECT_EQ(stereo, (std::vector<float>{0.3f, 0.3f, -0.1f, -0.1f}));
}

TEST(AudioConverterTest, ConvertChannelsStereoToMono) {
  const auto mono =
      AudioConverter::convert_channels({0.2f, 0.4f, -0.2f, 0.0f}, 2, 1);
  ASSERT_EQ(mono.size(), 2u);
  EXPECT_NEAR(mono[0], 0.3f, 1e-6f);
  EXPECT_NEAR(mono[1], -0.1f, 1e-6f);
}

TEST(AudioConverterTest, ConvertChannelsRejectsUnsupported) {
  EXPECT_THROW(AudioConverter::convert_channels({0.1f, 0.2f}, 0, 1),
               std::invalid_argument);
  EXPECT_THROW(AudioConverter::convert_channels({0.1f, 0.2f, 0.3f}, 2, 1),
               std::invalid_argument);
  EXPECT_THROW(AudioConverter::convert_channels(
                   {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f}, 3, 2),
               std::invalid_argument);
}

TEST(AudioConverterTest, ScalarAndFormatHelpers) {
  EXPECT_FLOAT_EQ(audio_common_dsp::int16_to_float(16384), 0.5f);
  EXPECT_FLOAT_EQ(audio_common_dsp::int16_to_float(-32768), -1.0f);
  EXPECT_TRUE(AudioConverter::is_supported_format(audio_common_dsp::kPaInt16));
  EXPECT_FALSE(AudioConverter::is_supported_format(999));
}
