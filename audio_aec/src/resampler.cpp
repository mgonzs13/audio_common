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

#include "audio_aec/resampler.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace audio_aec {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kCutoffRatio = 0.45;
constexpr double kButterworthQ1 = 0.54119610;
constexpr double kButterworthQ2 = 1.30656296;

} // namespace

Resampler::Resampler()
    : position_(0.0), input_rate_(0), output_rate_(0), filter_enabled_(false) {
  this->reset();
}

void Resampler::reset() {
  this->buffer_.clear();
  this->position_ = 0.0;
  this->input_rate_ = 0;
  this->output_rate_ = 0;
  this->filter_enabled_ = false;
  this->lowpass1_ = Biquad();
  this->lowpass2_ = Biquad();
}

Resampler::Biquad Resampler::make_lowpass(double cutoff_hz, double sample_rate,
                                          double q) {
  Biquad filter;
  const double w0 = 2.0 * kPi * cutoff_hz / sample_rate;
  const double cos_w0 = std::cos(w0);
  const double alpha = std::sin(w0) / (2.0 * q);
  const double a0 = 1.0 + alpha;

  filter.b0 = (1.0 - cos_w0) / 2.0 / a0;
  filter.b1 = (1.0 - cos_w0) / a0;
  filter.b2 = filter.b0;
  filter.a1 = -2.0 * cos_w0 / a0;
  filter.a2 = (1.0 - alpha) / a0;

  return filter;
}

float Resampler::apply_biquad(Biquad &filter, float sample) {
  const double input = sample;
  const double output = filter.b0 * input + filter.z1;

  // Transposed direct-form II.
  filter.z1 = filter.b1 * input - filter.a1 * output + filter.z2;
  filter.z2 = filter.b2 * input - filter.a2 * output;

  return static_cast<float>(output);
}

std::vector<float> Resampler::process(const std::vector<float> &input,
                                      int input_rate, int output_rate) {
  if (input_rate <= 0 || output_rate <= 0) {
    throw std::invalid_argument("Sample rates must be positive");
  }

  if (input_rate == output_rate) {
    this->reset();
    return input;
  }

  if (input_rate != this->input_rate_ || output_rate != this->output_rate_) {
    this->reset();
    this->input_rate_ = input_rate;
    this->output_rate_ = output_rate;
    this->filter_enabled_ = input_rate > output_rate;

    if (this->filter_enabled_) {
      const double cutoff = kCutoffRatio * static_cast<double>(output_rate);
      this->lowpass1_ = this->make_lowpass(cutoff, input_rate, kButterworthQ1);
      this->lowpass2_ = this->make_lowpass(cutoff, input_rate, kButterworthQ2);
    }
  }

  this->buffer_.reserve(this->buffer_.size() + input.size());
  for (float sample : input) {
    if (this->filter_enabled_) {
      sample = this->apply_biquad(this->lowpass1_, sample);
      sample = this->apply_biquad(this->lowpass2_, sample);
    }
    this->buffer_.push_back(sample);
  }

  std::vector<float> output;
  output.reserve(static_cast<std::size_t>(this->buffer_.size() * output_rate /
                                          input_rate) +
                 1);

  const double step = static_cast<double>(input_rate) / output_rate;
  while (this->position_ + 1.0 < static_cast<double>(this->buffer_.size())) {
    const std::size_t index = static_cast<std::size_t>(this->position_);
    const double fraction = this->position_ - static_cast<double>(index);
    output.push_back(
        static_cast<float>((1.0 - fraction) * this->buffer_[index] +
                           fraction * this->buffer_[index + 1]));
    this->position_ += step;
  }

  const std::size_t consumed = std::min<std::size_t>(
      static_cast<std::size_t>(this->position_), this->buffer_.size());
  if (consumed > 0) {
    this->buffer_.erase(this->buffer_.begin(),
                        this->buffer_.begin() + consumed);
    this->position_ -= static_cast<double>(consumed);
  }

  return output;
}

std::vector<float> Resampler::flush() {
  std::vector<float> output;

  if (this->buffer_.empty() || this->input_rate_ <= 0 ||
      this->output_rate_ <= 0) {
    this->reset();
    return output;
  }

  const double step =
      static_cast<double>(this->input_rate_) / this->output_rate_;
  const double end = static_cast<double>(this->buffer_.size());

  while (this->position_ < end) {
    const std::size_t index = static_cast<std::size_t>(this->position_);
    const double fraction = this->position_ - static_cast<double>(index);

    if (index + 1 < this->buffer_.size()) {
      output.push_back(
          static_cast<float>((1.0 - fraction) * this->buffer_[index] +
                             fraction * this->buffer_[index + 1]));
    } else {
      // No interpolation partner remains; hold the final input sample.
      output.push_back(this->buffer_[index]);
    }

    this->position_ += step;
  }

  this->reset();
  return output;
}

} // namespace audio_aec
