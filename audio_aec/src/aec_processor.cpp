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

#include "audio_common_dsp/sample_format.hpp"
#include "audio_common_dsp/time_utils.hpp"

namespace audio_aec {
namespace {

AecProcessorConfig validate_config(const AecProcessorConfig &config) {
  if (!WebrtcAec::is_supported_sample_rate(config.rate)) {
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
  if (config.reference_lead_ms < 0 || config.reference_lead_ms > 500) {
    throw std::invalid_argument(
        "AecProcessor reference_lead_ms must be between 0 and 500");
  }
  if (config.reference_wait_ms < 0 || config.reference_wait_ms > 1000) {
    throw std::invalid_argument(
        "AecProcessor reference_wait_ms must be between 0 and 1000");
  }
  if (config.reference_timeout_ms <= 0) {
    throw std::invalid_argument(
        "AecProcessor reference_timeout_ms must be positive");
  }

  return config;
}

} // namespace

bool AecProcessor::is_supported_rate(int rate) {
  return WebrtcAec::is_supported_sample_rate(rate);
}

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
      last_reference_stamp_ns_(0), last_reference_end_stamp_ns_(0),
      reference_timed_out_(false), reference_reset_count_(0) {}

void AecProcessor::push_mic(const std::vector<float> &mono, int input_rate,
                            int64_t stamp_ns) {
  if (this->last_reference_stamp_ns_ != 0 && !this->reference_timed_out_ &&
      stamp_ns - this->last_reference_stamp_ns_ >
          static_cast<int64_t>(this->config_.reference_timeout_ms) *
              1000000LL) {
    this->reference_fifo_.clear();
    this->reference_available_ = 0;
    this->last_reference_end_stamp_ns_ = 0;
    this->reference_resampler_.reset();
    this->aec_.reset();
    this->reference_timed_out_ = true;
    ++this->reference_reset_count_;
  }

  std::vector<float> sanitized = mono;
  audio_common_dsp::sanitize_samples(sanitized);
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
  // The reference is ordered by playback time. Drop a stale block rather than
  // letting an out-of-order arrival corrupt the timestamp mapping.
  if (this->last_reference_stamp_ns_ != 0 &&
      stamp_ns < this->last_reference_stamp_ns_) {
    return;
  }

  std::vector<float> sanitized = mono;
  audio_common_dsp::sanitize_samples(sanitized);
  const std::vector<float> resampled = this->reference_resampler_.process(
      sanitized, input_rate, this->config_.rate);
  if (!resampled.empty()) {
    TimedBlock block;
    block.samples = resampled;
    block.stamp_ns = stamp_ns;
    this->reference_available_ += block.samples.size();
    this->last_reference_end_stamp_ns_ =
        stamp_ns + static_cast<int64_t>(block.samples.size()) *
                       audio_common_dsp::kNanosecondsPerSecond /
                       this->config_.rate;
    this->reference_fifo_.push_back(std::move(block));
  }

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
    // Do not consume the microphone frame until the reference for its
    // timestamp (plus the configured lead) has arrived. The next reference
    // callback will retry.
    const int64_t target =
        this->mic_fifo_.front().stamp_ns +
        static_cast<int64_t>(this->config_.reference_lead_ms) * 1000000LL;
    if (!this->reference_ready(target)) {
      break;
    }
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
  if (!this->reference_timed_out_ && !reference_tail.empty()) {
    TimedBlock block;
    block.samples = reference_tail;
    block.stamp_ns = this->last_reference_end_stamp_ns_;
    this->reference_available_ += block.samples.size();
    this->last_reference_end_stamp_ns_ =
        block.stamp_ns + static_cast<int64_t>(block.samples.size()) *
                             audio_common_dsp::kNanosecondsPerSecond /
                             this->config_.rate;
    this->reference_fifo_.push_back(std::move(block));
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
  this->last_reference_end_stamp_ns_ = 0;
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
      block.stamp_ns += static_cast<int64_t>(take) *
                        audio_common_dsp::kNanosecondsPerSecond /
                        this->config_.rate;
    }
  }

  this->mic_available_ -= static_cast<std::size_t>(this->frame_size_);
  return frame;
}

int64_t AecProcessor::reference_end_stamp_ns() const {
  if (this->reference_fifo_.empty()) {
    return this->last_reference_end_stamp_ns_;
  }

  const TimedBlock &block = this->reference_fifo_.back();
  return block.stamp_ns +
         static_cast<int64_t>(block.samples.size()) *
             audio_common_dsp::kNanosecondsPerSecond / this->config_.rate;
}

bool AecProcessor::reference_ready(int64_t stamp_ns) const {
  if (this->reference_timed_out_ || this->last_reference_stamp_ns_ == 0) {
    return true;
  }

  const int64_t end = this->reference_end_stamp_ns();
  if (end == 0) {
    return true;
  }
  if (!this->reference_fifo_.empty() &&
      stamp_ns < this->reference_fifo_.front().stamp_ns) {
    // The requested time predates the oldest buffered reference. Waiting
    // cannot recover it; process the gap as silence.
    return true;
  }
  if (stamp_ns < end) {
    return true;
  }

  return stamp_ns - end >
         static_cast<int64_t>(this->config_.reference_wait_ms) * 1000000LL;
}

std::vector<float> AecProcessor::pop_reference_frame(int64_t stamp_ns) {
  std::vector<float> frame(static_cast<std::size_t>(this->frame_size_), 0.0f);
  if (this->reference_timed_out_) {
    return frame;
  }

  const int64_t ns_per_sample =
      audio_common_dsp::kNanosecondsPerSecond / this->config_.rate;

  // Discard reference audio that is entirely before the requested time, and
  // trim the leading part of the block that straddles it.
  while (!this->reference_fifo_.empty()) {
    TimedBlock &block = this->reference_fifo_.front();
    const int64_t block_end =
        block.stamp_ns +
        static_cast<int64_t>(block.samples.size()) * ns_per_sample;

    if (block_end <= stamp_ns) {
      this->reference_available_ -= block.samples.size();
      this->reference_fifo_.pop_front();
      continue;
    }

    const int64_t delta = stamp_ns - block.stamp_ns;
    if (delta <= 0) {
      break;
    }

    const std::size_t skip = static_cast<std::size_t>(
        delta * this->config_.rate / audio_common_dsp::kNanosecondsPerSecond);
    if (skip >= block.samples.size()) {
      this->reference_available_ -= block.samples.size();
      this->reference_fifo_.pop_front();
      continue;
    }
    block.samples.erase(block.samples.begin(), block.samples.begin() + skip);
    block.stamp_ns += static_cast<int64_t>(skip) * ns_per_sample;
    this->reference_available_ -= skip;
    break;
  }

  if (this->reference_fifo_.empty()) {
    return frame;
  }

  std::size_t offset = 0;
  const int64_t first_start = this->reference_fifo_.front().stamp_ns;
  if (first_start > stamp_ns) {
    // The reference begins inside this frame; leave the leading part silent.
    const int64_t gap =
        (first_start - stamp_ns) * this->config_.rate /
        audio_common_dsp::kNanosecondsPerSecond;
    if (gap >= this->frame_size_) {
      return frame;
    }
    offset = static_cast<std::size_t>(gap);
  }

  while (offset < frame.size() && !this->reference_fifo_.empty()) {
    TimedBlock &block = this->reference_fifo_.front();
    const std::size_t take =
        std::min(frame.size() - offset, block.samples.size());
    std::copy(block.samples.begin(), block.samples.begin() + take,
              frame.begin() + offset);
    block.samples.erase(block.samples.begin(), block.samples.begin() + take);
    block.stamp_ns += static_cast<int64_t>(take) * ns_per_sample;
    this->reference_available_ -= take;
    offset += take;

    if (block.samples.empty()) {
      this->reference_fifo_.pop_front();
    }
  }

  return frame;
}

bool AecProcessor::process_aec_frame(const float *reference, const float *mic,
                                     float *processed, int &error_code) {
  const size_t frames = static_cast<size_t>(this->frame_size_);
  this->aec_.set_delay_ms(this->config_.stream_delay_ms +
                          this->config_.reference_lead_ms);

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
  const int64_t reference_stamp_ns =
      stamp_ns + static_cast<int64_t>(this->config_.reference_lead_ms) *
                     1000000LL;
  const std::vector<float> reference_frame =
      this->pop_reference_frame(reference_stamp_ns);

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
    audio_common_dsp::sanitize_samples(processed);
    this->last_error_ = error_code;
    this->aec_.reset();
    this->had_error_ = true;
  } else if (!finite) {
    audio_common_dsp::sanitize_samples(processed);
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
                              audio_common_dsp::kNanosecondsPerSecond /
                              this->config_.rate;
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

  while (this->reference_available_ > this->reference_capacity_frames_ &&
         !this->reference_fifo_.empty()) {
    this->reference_available_ -= this->reference_fifo_.front().samples.size();
    this->reference_fifo_.pop_front();
  }

  this->aec_.reset();
  this->had_error_ = true;
}

int64_t AecProcessor::next_mic_stamp() const {
  if (!this->mic_fifo_.empty()) {
    const TimedBlock &block = this->mic_fifo_.back();
    return block.stamp_ns + static_cast<int64_t>(block.samples.size()) *
                                audio_common_dsp::kNanosecondsPerSecond /
                                this->config_.rate;
  }
  if (this->output_stamp_ns_ >= 0) {
    return this->output_stamp_ns_ +
           static_cast<int64_t>(this->output_samples_.size()) *
               audio_common_dsp::kNanosecondsPerSecond / this->config_.rate;
  }

  return 0;
}

} // namespace audio_aec
