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

#ifndef AUDIO_AEC__WEBRTC_AEC_HPP_
#define AUDIO_AEC__WEBRTC_AEC_HPP_

#include <cstddef>
#include <memory>

namespace audio_aec {

/**
 * @brief Mono acoustic echo canceller backed by WebRTC AudioProcessing.
 *
 * The wrapper processes 10 ms frames of deinterleaved float audio in the range
 * [-1, 1]. Echo cancellation is always enabled; the high-pass filter, noise
 * suppression, and automatic gain control can be toggled at construction.
 * WebRTC types are hidden behind a PIMPL so the public header stays free of
 * the system library headers.
 */
class WebrtcAec {
public:
  /**
   * @brief Create an echo canceller.
   *
   * @param sample_rate Sample rate in Hz; 8000, 16000, 32000, or 48000.
   * @param high_pass Enable the high-pass filter.
   * @param noise_suppression Enable high-level noise suppression.
   * @param agc Enable fixed-digital automatic gain control.
   * @throws std::invalid_argument if @p sample_rate is not supported.
   */
  WebrtcAec(int sample_rate, bool high_pass, bool noise_suppression, bool agc);

  /// @brief Destroy the wrapper and release the WebRTC implementation.
  ~WebrtcAec();

  /// @brief Copy construction is disabled because the wrapper owns a PIMPL.
  WebrtcAec(const WebrtcAec &) = delete;

  /// @brief Copy assignment is disabled because the wrapper owns a PIMPL.
  WebrtcAec &operator=(const WebrtcAec &) = delete;

  /// @brief True if @p sample_rate is supported by WebRTC AudioProcessing.
  static bool is_supported_sample_rate(int sample_rate);

  /**
   * @brief Feed one far-end (render) frame to the echo canceller.
   *
   * @param frame Mono samples; must contain frame_size() samples.
   * @param frames Number of samples in @p frame.
   * @return True on success, false if @p frames is not frame_size().
   */
  bool process_render(const float *frame, size_t frames);

  /**
   * @brief Process one near-end (capture) frame.
   *
   * @param input Mono input samples; must contain frame_size() samples.
   * @param output Mono output buffer; must contain frame_size() samples.
   * @param frames Number of samples in @p input and @p output.
   * @return True on success, false if @p frames is not frame_size().
   */
  bool process_capture(const float *input, float *output, size_t frames);

  /// @brief Set the render-to-capture delay reported to WebRTC, in ms.
  void set_delay_ms(int delay_ms);

  /// @brief Reset the internal state while retaining the configuration.
  void reset();

  /// @brief Configured sample rate in Hz.
  int sample_rate() const;

  /// @brief Number of samples in one 10 ms frame.
  int frame_size() const;

  /**
   * @brief Return code from the most recent WebRTC processing call.
   *
   * The value is 0 when the most recent call succeeded, a negative
   * @c webrtc::AudioProcessing::Error code when WebRTC rejected the frame,
   * and 0 again after reset(). A frame-size mismatch is reported separately
   * by the boolean return value and does not change this code.
   */
  int last_error() const;

private:
  /// @brief Hidden WebRTC AudioProcessing implementation and its state.
  class Impl;

  /// @brief Owned WebRTC implementation; never null after construction.
  std::unique_ptr<Impl> impl_;
};

} // namespace audio_aec

#endif // AUDIO_AEC__WEBRTC_AEC_HPP_
