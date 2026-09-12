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

#include "audio_common/sample_rate_converter.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

using namespace audio_common;

std::vector<float>
SampleRateConverter::convert(const std::vector<float> &input_data, int channels,
                             int input_rate, int output_rate) {

  if (channels <= 0) {
    throw std::invalid_argument("channels must be positive");
  }
  if (input_rate <= 0 || output_rate <= 0) {
    throw std::invalid_argument("sample rates must be positive");
  }
  if (input_data.size() % static_cast<size_t>(channels) != 0) {
    throw std::invalid_argument("input data must contain complete frames");
  }

  std::vector<float> output_data;
  const size_t input_frames = input_data.size() / channels;
  if (input_frames == 0) {
    return output_data;
  }

  output_data.reserve(
      static_cast<size_t>(std::ceil(static_cast<double>(input_frames) *
                                    output_rate / input_rate)) *
      channels);

  std::vector<float> source_data;
  source_data.reserve(input_data.size() + this->previous_frame_.size());

  if (!this->previous_frame_.empty()) {
    source_data.insert(source_data.end(), this->previous_frame_.begin(),
                       this->previous_frame_.end());
  }

  source_data.insert(source_data.end(), input_data.begin(), input_data.end());

  const size_t source_frames = source_data.size() / channels;
  const double step = static_cast<double>(input_rate) / output_rate;

  while (this->position_ + 1.0 < static_cast<double>(source_frames)) {
    const size_t frame_index = static_cast<size_t>(this->position_);
    const double fraction = this->position_ - frame_index;

    for (int channel = 0; channel < channels; ++channel) {
      const float current_sample =
          source_data[frame_index * channels + channel];
      const float next_sample =
          source_data[(frame_index + 1) * channels + channel];
      output_data.push_back(static_cast<float>(
          current_sample + (next_sample - current_sample) * fraction));
    }

    this->position_ += step;
  }

  this->previous_frame_.assign(input_data.end() - channels, input_data.end());
  this->position_ -= static_cast<double>(source_frames - 1);

  return output_data;
}

std::vector<float> SampleRateConverter::flush() {
  std::vector<float> output_data = std::move(this->previous_frame_);
  this->position_ = 0.0;
  return output_data;
}