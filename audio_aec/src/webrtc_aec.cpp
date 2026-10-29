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

#include "audio_aec/webrtc_aec.hpp"

#ifdef AUDIO_AEC_WEBRTC_NEW_API
#include <modules/audio_processing/include/audio_processing.h>
#else
#include <webrtc/modules/audio_processing/include/audio_processing.h>
#endif

#include <memory>
#include <stdexcept>
#include <vector>

namespace audio_aec {
namespace {

int validate_sample_rate(int sample_rate) {
  if (!WebrtcAec::is_supported_sample_rate(sample_rate)) {
    throw std::invalid_argument(
        "WebRTC AudioProcessing supports only 8000, 16000, 32000, or 48000 Hz");
  }
  return sample_rate;
}

} // namespace

bool WebrtcAec::is_supported_sample_rate(int sample_rate) {
  return sample_rate == 8000 || sample_rate == 16000 || sample_rate == 32000 ||
         sample_rate == 48000;
}

class WebrtcAec::Impl {
public:
  Impl(int sample_rate, bool high_pass, bool noise_suppression, bool agc);

  bool process_render(const float *frame, size_t frames);
  bool process_capture(const float *input, float *output, size_t frames);

  int sample_rate_;
  int frame_size_;
  int delay_ms_;
  int last_error_;
  std::vector<float> render_scratch_;
  std::unique_ptr<webrtc::AudioProcessing> apm_;
};

WebrtcAec::Impl::Impl(int sample_rate, bool high_pass, bool noise_suppression,
                      bool agc)
    : sample_rate_(validate_sample_rate(sample_rate)),
      frame_size_(this->sample_rate_ / 100), delay_ms_(0), last_error_(0),
      render_scratch_(static_cast<size_t>(this->frame_size_), 0.0f) {
#ifdef AUDIO_AEC_WEBRTC_NEW_API
  this->apm_.reset(webrtc::AudioProcessingBuilder().Create());
  if (!this->apm_) {
    throw std::runtime_error("Failed to create WebRTC AudioProcessing");
  }

  webrtc::AudioProcessing::Config config;
  config.echo_canceller.enabled = true;
  config.high_pass_filter.enabled = high_pass;
  config.noise_suppression.enabled = noise_suppression;
  if (noise_suppression) {
    config.noise_suppression.level =
        webrtc::AudioProcessing::Config::NoiseSuppression::kHigh;
  }
  config.gain_controller1.enabled = agc;
  if (agc) {
    config.gain_controller1.mode =
        webrtc::AudioProcessing::Config::GainController1::kFixedDigital;
  }
  this->apm_->ApplyConfig(config);
#else
  webrtc::Config config;
  config.Set<webrtc::DelayAgnostic>(new webrtc::DelayAgnostic(true));
  config.Set<webrtc::ExtendedFilter>(new webrtc::ExtendedFilter(true));

  this->apm_.reset(webrtc::AudioProcessing::Create(config));
  if (!this->apm_) {
    throw std::runtime_error("Failed to create WebRTC AudioProcessing");
  }

  this->apm_->echo_cancellation()->Enable(true);
  this->apm_->echo_cancellation()->set_suppression_level(
      webrtc::EchoCancellation::kHighSuppression);

  if (high_pass) {
    this->apm_->high_pass_filter()->Enable(true);
  }
  if (noise_suppression) {
    this->apm_->noise_suppression()->set_level(webrtc::NoiseSuppression::kHigh);
    this->apm_->noise_suppression()->Enable(true);
  }
  if (agc) {
    this->apm_->gain_control()->set_mode(webrtc::GainControl::kFixedDigital);
    this->apm_->gain_control()->Enable(true);
  }
#endif
}

bool WebrtcAec::Impl::process_render(const float *frame, size_t frames) {
  if (frames != static_cast<size_t>(this->frame_size_)) {
    return false;
  }

  const webrtc::StreamConfig stream_config(this->sample_rate_, 1);
  const float *src[1] = {frame};
  float *dst[1] = {this->render_scratch_.data()};
  const int error =
      this->apm_->ProcessReverseStream(src, stream_config, stream_config, dst);
  this->last_error_ = error;

  return error == 0;
}

bool WebrtcAec::Impl::process_capture(const float *input, float *output,
                                      size_t frames) {
  if (frames != static_cast<size_t>(this->frame_size_)) {
    return false;
  }

  this->apm_->set_stream_delay_ms(this->delay_ms_);

  const webrtc::StreamConfig stream_config(this->sample_rate_, 1);
  const float *src[1] = {input};
  float *dst[1] = {output};
  const int error =
      this->apm_->ProcessStream(src, stream_config, stream_config, dst);
  this->last_error_ = error;

  return error == 0;
}

WebrtcAec::WebrtcAec(int sample_rate, bool high_pass, bool noise_suppression,
                     bool agc)
    : impl_(new Impl(sample_rate, high_pass, noise_suppression, agc)) {}

WebrtcAec::~WebrtcAec() = default;

bool WebrtcAec::process_render(const float *frame, size_t frames) {
  return this->impl_->process_render(frame, frames);
}

bool WebrtcAec::process_capture(const float *input, float *output,
                                size_t frames) {
  return this->impl_->process_capture(input, output, frames);
}

void WebrtcAec::set_delay_ms(int delay_ms) {
  this->impl_->delay_ms_ = delay_ms;
}

void WebrtcAec::reset() {
  this->impl_->apm_->Initialize();
  this->impl_->last_error_ = 0;
}

int WebrtcAec::sample_rate() const { return this->impl_->sample_rate_; }

int WebrtcAec::frame_size() const { return this->impl_->frame_size_; }

int WebrtcAec::last_error() const { return this->impl_->last_error_; }

} // namespace audio_aec
