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

#include "audio_common_dsp/multi_channel_resampler.hpp"

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace audio_common_dsp {

std::vector<float>
MultiChannelResampler::process(const std::vector<float> &input, int channels,
                               int input_rate, int output_rate) {
  if (channels <= 0) {
    throw std::invalid_argument("Channel count must be positive");
  }

  if (input.size() % static_cast<std::size_t>(channels) != 0) {
    throw std::invalid_argument(
        "Audio data length is not a multiple of the channel count");
  }

  if (channels != this->channels_) {
    this->channel_resamplers_.clear();
    this->channel_resamplers_.resize(static_cast<std::size_t>(channels));
    this->channels_ = channels;
  }

  const std::size_t frames = input.size() / static_cast<std::size_t>(channels);
  if (frames == 0) {
    return {};
  }

  std::vector<std::vector<float>> channel_inputs(
      static_cast<std::size_t>(channels), std::vector<float>(frames));
  for (std::size_t frame = 0; frame < frames; ++frame) {
    for (int channel = 0; channel < channels; ++channel) {
      channel_inputs[static_cast<std::size_t>(channel)][frame] =
          input[frame * static_cast<std::size_t>(channels) +
                static_cast<std::size_t>(channel)];
    }
  }

  std::size_t output_frames = 0;
  for (int channel = 0; channel < channels; ++channel) {
    std::vector<float> output =
        this->channel_resamplers_[static_cast<std::size_t>(channel)].process(
            channel_inputs[static_cast<std::size_t>(channel)], input_rate,
            output_rate);
    channel_inputs[static_cast<std::size_t>(channel)] = std::move(output);

    if (channel == 0) {
      output_frames = channel_inputs[static_cast<std::size_t>(channel)].size();
    } else {
      output_frames =
          std::min(output_frames,
                   channel_inputs[static_cast<std::size_t>(channel)].size());
    }
  }

  std::vector<float> interleaved(output_frames *
                                 static_cast<std::size_t>(channels));
  for (std::size_t frame = 0; frame < output_frames; ++frame) {
    for (int channel = 0; channel < channels; ++channel) {
      interleaved[frame * static_cast<std::size_t>(channels) +
                  static_cast<std::size_t>(channel)] =
          channel_inputs[static_cast<std::size_t>(channel)][frame];
    }
  }

  return interleaved;
}

std::vector<float> MultiChannelResampler::flush() {
  if (this->channels_ <= 0 || this->channel_resamplers_.empty()) {
    return {};
  }

  std::vector<std::vector<float>> channel_outputs(
      static_cast<std::size_t>(this->channels_));
  std::size_t output_frames = 0;

  for (int channel = 0; channel < this->channels_; ++channel) {
    channel_outputs[static_cast<std::size_t>(channel)] =
        this->channel_resamplers_[static_cast<std::size_t>(channel)].flush();

    if (channel == 0) {
      output_frames = channel_outputs[static_cast<std::size_t>(channel)].size();
    } else {
      output_frames =
          std::min(output_frames,
                   channel_outputs[static_cast<std::size_t>(channel)].size());
    }
  }

  std::vector<float> interleaved(output_frames *
                                 static_cast<std::size_t>(this->channels_));
  for (std::size_t frame = 0; frame < output_frames; ++frame) {
    for (int channel = 0; channel < this->channels_; ++channel) {
      interleaved[frame * static_cast<std::size_t>(this->channels_) +
                  static_cast<std::size_t>(channel)] =
          channel_outputs[static_cast<std::size_t>(channel)][frame];
    }
  }

  return interleaved;
}

void MultiChannelResampler::reset() {
  for (Resampler &resampler : this->channel_resamplers_) {
    resampler.reset();
  }
  this->channel_resamplers_.clear();
  this->channels_ = 0;
}

int MultiChannelResampler::channels() const { return this->channels_; }

} // namespace audio_common_dsp
