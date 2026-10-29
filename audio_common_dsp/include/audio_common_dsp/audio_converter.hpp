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

#ifndef AUDIO_COMMON_DSP__AUDIO_CONVERTER_HPP_
#define AUDIO_COMMON_DSP__AUDIO_CONVERTER_HPP_

#include <vector>

#include "audio_common_dsp/sample_format.hpp"
#include "audio_common_msgs/msg/audio_data.hpp"

namespace audio_common_dsp {

/**
 * @brief Converts audio_common_msgs::msg::AudioData between interleaved
 *        integer/float encodings and float samples.
 */
class AudioConverter {
public:
  /// @brief True if @p format is one of the supported PortAudio constants.
  static bool is_supported_format(int format);

  /**
   * @brief Convert interleaved audio data to interleaved float samples.
   *
   * Channels are preserved. Float32 samples are clamped to [-1, 1] and
   * non-finite samples (NaN and +/-Inf) are replaced with 0.
   *
   * @param data Interleaved audio data. Only the array matching @p format is
   *        read.
   * @param format One of the kPa* sample format constants.
   * @param channels Number of interleaved channels; must be positive.
   * @return Interleaved float samples in [-1, 1].
   * @throws std::invalid_argument if @p channels is not positive, @p format is
   *         unsupported, or the selected array length is not a multiple of
   *         @p channels.
   */
  static std::vector<float>
  to_float(const audio_common_msgs::msg::AudioData &data, int format,
           int channels);

  /**
   * @brief Convert interleaved float samples to interleaved audio data.
   *
   * Integer output is clamped to [-1, 1] before scaling. Float32 output is
   * clamped to [-1, 1] with non-finite samples replaced by 0. Only the array
   * matching @p format is written; the other arrays are left untouched.
   *
   * @param samples Interleaved float samples.
   * @param format One of the kPa* sample format constants.
   * @param[out] out Audio data whose matching array is replaced.
   * @throws std::invalid_argument if @p format is unsupported.
   */
  static void from_float(const std::vector<float> &samples, int format,
                         audio_common_msgs::msg::AudioData &out);

  /**
   * @brief Convert an interleaved signal between channel counts.
   *
   * Supports identical channel counts, mono to N channels (duplicate), and N
   * channels to mono (average). Other conversions throw.
   *
   * @param samples Interleaved float samples.
   * @param input_channels Number of interleaved channels in @p samples.
   * @param output_channels Desired number of interleaved channels.
   * @return Converted interleaved samples.
   * @throws std::invalid_argument if a channel count is not positive, the input
   *         length is not a multiple of @p input_channels, or the conversion is
   *         unsupported.
   */
  static std::vector<float> convert_channels(const std::vector<float> &samples,
                                             int input_channels,
                                             int output_channels);

  /**
   * @brief Convert interleaved audio data to mono float samples in [-1, 1].
   *
   * Multi-channel input is downmixed by averaging all channels of each frame.
   * Float32 samples are clamped to [-1, 1] and non-finite samples (NaN and
   * +/-Inf) are replaced with 0.
   *
   * @param data Interleaved audio data. Only the array matching @p format is
   *        read.
   * @param format One of the kPa* sample format constants.
   * @param channels Number of interleaved channels; must be positive.
   * @return Mono float samples in [-1, 1].
   * @throws std::invalid_argument if @p channels is not positive, @p format is
   *         unsupported, or the selected array length is not a multiple of
   *         @p channels.
   */
  static std::vector<float>
  to_mono_float(const audio_common_msgs::msg::AudioData &data, int format,
                int channels);

  /**
   * @brief Convert mono float samples to interleaved audio data.
   *
   * Integer output is clamped to [-1, 1] before scaling. Float32 output is
   * clamped to [-1, 1] with non-finite samples replaced by 0. Only the array
   * matching @p format is written; the other arrays are left untouched.
   *
   * @param samples Mono float samples.
   * @param format One of the kPa* sample format constants.
   * @param[out] out Audio data whose matching array is replaced.
   * @throws std::invalid_argument if @p format is unsupported.
   */
  static void from_mono_float(const std::vector<float> &samples, int format,
                              audio_common_msgs::msg::AudioData &out);
};

} // namespace audio_common_dsp

#endif // AUDIO_COMMON_DSP__AUDIO_CONVERTER_HPP_
