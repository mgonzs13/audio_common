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

#include "audio_aec/aec_processor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace audio_aec {
namespace {

constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

bool is_supported_rate(int rate) {
  return rate == 8000 || rate == 16000 || rate == 32000 || rate == 48000;
}

void sanitize_samples(std::vector<float> &samples) {
  for (float &sample : samples) {
    if (!std::isfinite(sample)) {
      sample = 0.0f;
    } else {
      sample = std::min(1.0f, std::max(-1.0f, sample));
    }
  }
}

AecProcessorConfig validate_config(const AecProcessorConfig &config) {
  if (!is_supported_rate(config.rate)) {
    throw std::invalid_argument(
        "AecProcessor rate must be 8000, 16000, 32000, or 48000");
  }
  if (config.mic_buffer_ms < 0 || config.mic_buffer_ms % 10 != 0 ||
      config.mic_buffer_ms > 500) {
    throw std::invalid_argument(
        "AecProcessor mic_buffer_ms must be non-negative, a multiple of 10, "
        "and at most 500");
  }
  if (config.output_chunk_ms <= 0 || config.output_chunk_ms % 10 != 0) {
    throw std::invalid_argument(
        "AecProcessor output_chunk_ms must be positive and a multiple of 10");
  }
  if (config.stream_delay_ms < 0) {
    throw std::invalid_argument(
        "AecProcessor stream_delay_ms must be non-negative");
  }
  if (config.reference_timeout_ms <= 0) {
    throw std::invalid_argument(
        "AecProcessor reference_timeout_ms must be positive");
  }

  return config;
}

} // namespace

AecProcessor::AecProcessor(const AecProcessorConfig &config)
    : config_(validate_config(config)), frame_size_(config_.rate / 100),
      mic_buffer_frames_(static_cast<int>(static_cast<int64_t>(config_.rate) *
                                          config_.mic_buffer_ms / 1000)),
      output_chunk_frames_(static_cast<int>(static_cast<int64_t>(config_.rate) *
                                            config_.output_chunk_ms / 1000)),
      mic_capacity_frames_(static_cast<std::size_t>(config_.rate)),
      reference_capacity_frames_(static_cast<std::size_t>(config_.rate)),
      aec_(config_.rate, config_.enable_high_pass,
           config_.enable_noise_suppression, config_.enable_agc),
      mic_resampler_(), reference_resampler_(), mic_fifo_(), reference_fifo_(),
      mic_available_(0), reference_available_(0), output_samples_(),
      output_stamp_ns_(-1), had_error_(false), last_error_(0),
      last_reference_stamp_ns_(0), reference_timed_out_(false),
      reference_reset_count_(0) {}

void AecProcessor::push_mic(const std::vector<float> &mono, int input_rate,
                            int64_t stamp_ns) {
  if (this->last_reference_stamp_ns_ != 0 && !this->reference_timed_out_ &&
      stamp_ns - this->last_reference_stamp_ns_ >
          static_cast<int64_t>(this->config_.reference_timeout_ms) *
              1000000LL) {
    this->reference_fifo_.clear();
    this->reference_available_ = 0;
    this->reference_resampler_.reset();
    this->aec_.reset();
    this->reference_timed_out_ = true;
    ++this->reference_reset_count_;
  }

  std::vector<float> sanitized = mono;
  sanitize_samples(sanitized);
  std::vector<float> resampled =
      this->mic_resampler_.process(sanitized, input_rate, this->config_.rate);
  if (resampled.empty()) {
    return;
  }

  TimedBlock block;
  block.samples = std::move(resampled);
  block.stamp_ns = stamp_ns;
  this->mic_available_ += block.samples.size();
  this->mic_fifo_.push_back(std::move(block));

  this->enforce_mic_capacity();
}

void AecProcessor::push_reference(const std::vector<float> &mono,
                                  int input_rate, int64_t stamp_ns) {
  std::vector<float> sanitized = mono;
  sanitize_samples(sanitized);
  const std::vector<float> resampled = this->reference_resampler_.process(
      sanitized, input_rate, this->config_.rate);
  this->reference_fifo_.insert(this->reference_fifo_.end(), resampled.begin(),
                               resampled.end());
  this->reference_available_ += resampled.size();

  if (stamp_ns > this->last_reference_stamp_ns_) {
    this->last_reference_stamp_ns_ = stamp_ns;
  }
  this->reference_timed_out_ = false;

  this->enforce_reference_capacity();
}

std::vector<OutputChunk> AecProcessor::pop_output() {
  std::vector<OutputChunk> output;

  while (
      this->mic_available_ >=
      static_cast<std::size_t>(this->mic_buffer_frames_ + this->frame_size_)) {
    this->process_next_frame(output);
  }

  return output;
}

std::vector<OutputChunk> AecProcessor::flush() {
  std::vector<OutputChunk> output;

  const std::vector<float> mic_tail = this->mic_resampler_.flush();
  if (!mic_tail.empty()) {
    TimedBlock block;
    block.samples = mic_tail;
    block.stamp_ns = this->next_mic_stamp();
    this->mic_available_ += block.samples.size();
    this->mic_fifo_.push_back(std::move(block));
  }

  const std::vector<float> reference_tail = this->reference_resampler_.flush();
  if (!this->reference_timed_out_) {
    this->reference_fifo_.insert(this->reference_fifo_.end(),
                                 reference_tail.begin(), reference_tail.end());
    this->reference_available_ += reference_tail.size();
  }

  while (this->mic_available_ >= static_cast<std::size_t>(this->frame_size_)) {
    this->process_next_frame(output);
  }

  if (!this->output_samples_.empty()) {
    OutputChunk chunk;
    chunk.samples = std::move(this->output_samples_);
    chunk.stamp_ns = this->output_stamp_ns_;
    this->output_samples_.clear();
    this->output_stamp_ns_ = -1;
    output.push_back(std::move(chunk));
  }

  return output;
}

void AecProcessor::reset() {
  this->mic_fifo_.clear();
  this->reference_fifo_.clear();
  this->mic_available_ = 0;
  this->reference_available_ = 0;
  this->mic_resampler_.reset();
  this->reference_resampler_.reset();
  this->output_samples_.clear();
  this->output_stamp_ns_ = -1;
  this->aec_.reset();
  this->last_reference_stamp_ns_ = 0;
  this->reference_timed_out_ = false;
}

bool AecProcessor::had_error() const { return this->had_error_; }

void AecProcessor::clear_error() {
  this->had_error_ = false;
  this->last_error_ = 0;
}

int AecProcessor::last_error() const { return this->last_error_; }

uint64_t AecProcessor::reference_reset_count() const {
  return this->reference_reset_count_;
}

int AecProcessor::rate() const { return this->config_.rate; }

int AecProcessor::frame_size() const { return this->frame_size_; }

std::vector<float> AecProcessor::pop_mic_frame(int64_t &stamp_ns) {
  std::vector<float> frame;
  frame.reserve(static_cast<std::size_t>(this->frame_size_));
  stamp_ns = this->mic_fifo_.front().stamp_ns;

  while (frame.size() < static_cast<std::size_t>(this->frame_size_)) {
    TimedBlock &block = this->mic_fifo_.front();
    const std::size_t needed =
        static_cast<std::size_t>(this->frame_size_) - frame.size();
    const std::size_t take = std::min(needed, block.samples.size());

    frame.insert(frame.end(), block.samples.begin(),
                 block.samples.begin() + take);
    block.samples.erase(block.samples.begin(), block.samples.begin() + take);

    if (block.samples.empty()) {
      this->mic_fifo_.pop_front();
    } else {
      block.stamp_ns += static_cast<int64_t>(take) * kNanosecondsPerSecond /
                        this->config_.rate;
    }
  }

  this->mic_available_ -= static_cast<std::size_t>(this->frame_size_);
  return frame;
}

std::vector<float> AecProcessor::pop_reference_frame() {
  std::vector<float> frame(static_cast<std::size_t>(this->frame_size_), 0.0f);
  if (this->reference_timed_out_) {
    return frame;
  }

  std::size_t filled = 0;

  while (filled < frame.size() && !this->reference_fifo_.empty()) {
    frame[filled] = this->reference_fifo_.front();
    this->reference_fifo_.pop_front();
    ++filled;
  }

  this->reference_available_ -= filled;
  return frame;
}

bool AecProcessor::process_aec_frame(const float *reference, const float *mic,
                                     float *processed, int &error_code) {
  const size_t frames = static_cast<size_t>(this->frame_size_);
  this->aec_.set_delay_ms(this->config_.stream_delay_ms);

  int code = 0;
  const bool rendered = this->aec_.process_render(reference, frames);
  if (!rendered) {
    code = this->aec_.last_error();
  }
  const bool captured = this->aec_.process_capture(mic, processed, frames);
  if (!captured) {
    code = this->aec_.last_error();
  }

  if (!rendered || !captured) {
    error_code = code;
    return false;
  }

  return true;
}

void AecProcessor::process_next_frame(std::vector<OutputChunk> &output) {
  int64_t stamp_ns = 0;
  const std::vector<float> mic_frame = this->pop_mic_frame(stamp_ns);
  const std::vector<float> reference_frame = this->pop_reference_frame();

  std::vector<float> processed(static_cast<std::size_t>(this->frame_size_),
                               0.0f);
  int error_code = 0;
  const bool processed_ok = this->process_aec_frame(
      reference_frame.data(), mic_frame.data(), processed.data(), error_code);

  bool finite = true;
  for (float sample : processed) {
    if (!std::isfinite(sample)) {
      finite = false;
      break;
    }
  }

  if (!processed_ok) {
    processed = mic_frame;
    sanitize_samples(processed);
    this->last_error_ = error_code;
    this->aec_.reset();
    this->had_error_ = true;
  } else if (!finite) {
    sanitize_samples(processed);
    this->last_error_ = 0;
    this->aec_.reset();
    this->mic_resampler_.reset();
    this->reference_resampler_.reset();
    this->had_error_ = true;
  }

  if (this->output_samples_.empty()) {
    this->output_stamp_ns_ = stamp_ns;
  }
  this->output_samples_.insert(this->output_samples_.end(), processed.begin(),
                               processed.end());

  while (this->output_samples_.size() >=
         static_cast<std::size_t>(this->output_chunk_frames_)) {
    OutputChunk chunk;
    chunk.samples.assign(this->output_samples_.begin(),
                         this->output_samples_.begin() +
                             this->output_chunk_frames_);
    chunk.stamp_ns = this->output_stamp_ns_;
    this->output_samples_.erase(this->output_samples_.begin(),
                                this->output_samples_.begin() +
                                    this->output_chunk_frames_);
    this->output_stamp_ns_ += static_cast<int64_t>(this->output_chunk_frames_) *
                              kNanosecondsPerSecond / this->config_.rate;
    output.push_back(std::move(chunk));
  }

  if (this->output_samples_.empty()) {
    this->output_stamp_ns_ = -1;
  }
}

void AecProcessor::enforce_mic_capacity() {
  if (this->mic_available_ <= this->mic_capacity_frames_) {
    return;
  }

  while (this->mic_available_ > this->mic_capacity_frames_ &&
         !this->mic_fifo_.empty()) {
    this->mic_available_ -= this->mic_fifo_.front().samples.size();
    this->mic_fifo_.pop_front();
  }

  this->aec_.reset();
  this->had_error_ = true;
}

void AecProcessor::enforce_reference_capacity() {
  if (this->reference_available_ <= this->reference_capacity_frames_) {
    return;
  }

  const std::size_t excess =
      this->reference_available_ - this->reference_capacity_frames_;
  for (std::size_t i = 0; i < excess; ++i) {
    this->reference_fifo_.pop_front();
  }
  this->reference_available_ -= excess;

  this->aec_.reset();
  this->had_error_ = true;
}

int64_t AecProcessor::next_mic_stamp() const {
  if (!this->mic_fifo_.empty()) {
    const TimedBlock &block = this->mic_fifo_.back();
    return block.stamp_ns + static_cast<int64_t>(block.samples.size()) *
                                kNanosecondsPerSecond / this->config_.rate;
  }
  if (this->output_stamp_ns_ >= 0) {
    return this->output_stamp_ns_ +
           static_cast<int64_t>(this->output_samples_.size()) *
               kNanosecondsPerSecond / this->config_.rate;
  }

  return 0;
}

} // namespace audio_aec
