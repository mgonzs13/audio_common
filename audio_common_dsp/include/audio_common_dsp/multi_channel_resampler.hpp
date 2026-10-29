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

#ifndef AUDIO_COMMON_DSP__MULTI_CHANNEL_RESAMPLER_HPP_
#define AUDIO_COMMON_DSP__MULTI_CHANNEL_RESAMPLER_HPP_

#include <vector>

#include "audio_common_dsp/resampler.hpp"

namespace audio_common_dsp {

/**
 * @brief Streaming interleaved multichannel sample-rate converter.
 *
 * Each channel is resampled independently with an anti-alias Resampler and the
 * results are interleaved again. The channel count is taken from the first
 * process() call and may change between calls; changing it resets the internal
 * channel state.
 */
class MultiChannelResampler {
public:
  /**
   * @brief Resample one interleaved block.
   *
   * @param input Interleaved float samples.
   * @param channels Number of interleaved channels; must be positive.
   * @param input_rate Input sample rate in Hz.
   * @param output_rate Output sample rate in Hz.
   * @return Interleaved resampled samples.
   * @throws std::invalid_argument if a rate or channel count is not positive,
   *         or the input length is not a multiple of @p channels.
   */
  std::vector<float> process(const std::vector<float> &input, int channels,
                             int input_rate, int output_rate);

  /**
   * @brief Drain the remaining output-rate samples from every channel.
   *
   * @return Interleaved remaining output-rate samples.
   */
  std::vector<float> flush();

  /// @brief Clear all channel resamplers and the remembered channel count.
  void reset();

  /// @brief Number of channels remembered from the last process() call.
  int channels() const;

private:
  /// @brief One independent resampler per interleaved channel.
  std::vector<Resampler> channel_resamplers_;

  /// @brief Channel count remembered from the last process() call; 0 before
  /// the first call.
  int channels_ = 0;
};

} // namespace audio_common_dsp

#endif // AUDIO_COMMON_DSP__MULTI_CHANNEL_RESAMPLER_HPP_
