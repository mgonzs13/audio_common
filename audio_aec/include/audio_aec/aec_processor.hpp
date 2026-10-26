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

#ifndef AUDIO_AEC__AEC_PROCESSOR_HPP_
#define AUDIO_AEC__AEC_PROCESSOR_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include "audio_aec/webrtc_aec.hpp"
#include "audio_common_dsp/resampler.hpp"

namespace audio_aec {

/// @brief Configuration for AecProcessor.
struct AecProcessorConfig {
  /// @brief Internal processing and output sample rate in Hz.
  int rate = 16000;
  /// @brief Microphone delay that gives reference audio time to arrive, in ms.
  int mic_buffer_ms = 100;
  /// @brief Duration of each returned output chunk, in ms.
  int output_chunk_ms = 20;
  /// @brief Initial render-to-capture delay hint passed to WebRTC, in ms.
  int stream_delay_ms = 0;
  /// @brief Enable WebRTC's high-pass filter.
  bool enable_high_pass = true;
  /// @brief Enable WebRTC noise suppression.
  bool enable_noise_suppression = false;
  /// @brief Enable WebRTC automatic gain control.
  bool enable_agc = false;
  /// @brief Silence interval after which the reference is considered gone, in
  /// ms.
  int reference_timeout_ms = 500;
};

/// @brief One block of cleaned audio with the capture stamp of its first
/// sample.
struct OutputChunk {
  std::vector<float> samples;
  int64_t stamp_ns = 0;
};

/**
 * @brief Buffered acoustic echo canceller for mono float audio.
 *
 * Microphone and reference audio may arrive at any input rate; both are
 * resampled to the configured processing rate. Microphone audio is held for
 * @c mic_buffer_ms so that the matching reference audio is available before
 * processing. Audio is processed in 10 ms frames and returned in chunks of
 * @c output_chunk_ms.
 *
 * The class is not internally synchronized; callers sharing an instance
 * between threads must provide their own locking.
 */
class AecProcessor {
public:
  /**
   * @brief Create a processor.
   *
   * @param config Processing configuration.
   * @throws std::invalid_argument if @c rate is not 8000, 16000, 32000, or
   *         48000; @c mic_buffer_ms is negative, not a multiple of 10, or
   *         larger than 500; @c output_chunk_ms is not positive or not a
   *         multiple of 10; @c stream_delay_ms is negative; or
   *         @c reference_timeout_ms is not positive.
   */
  explicit AecProcessor(const AecProcessorConfig &config);

  /**
   * @brief Append microphone audio.
   *
   * @param mono Mono float samples in [-1, 1].
   * @param input_rate Sample rate of @p mono in Hz.
   * @param stamp_ns Capture timestamp of the first sample in @p mono.
   * @throws std::invalid_argument if @p input_rate is not positive.
   */
  void push_mic(const std::vector<float> &mono, int input_rate,
                int64_t stamp_ns);

  /**
   * @brief Append far-end reference audio.
   *
   * @param mono Mono float samples in [-1, 1].
   * @param input_rate Sample rate of @p mono in Hz.
   * @param stamp_ns Capture timestamp of the first sample in @p mono.
   * @throws std::invalid_argument if @p input_rate is not positive.
   */
  void push_reference(const std::vector<float> &mono, int input_rate,
                      int64_t stamp_ns);

  /**
   * @brief Process buffered audio and return every completed output chunk.
   *
   * @return Output chunks whose timestamps identify their first sample.
   */
  std::vector<OutputChunk> pop_output();

  /**
   * @brief Drain the resamplers and return all remaining audio.
   *
   * Buffered microphone frames are processed even if less than
   * @c mic_buffer_ms remains. The final output is returned as a partial chunk
   * when it is shorter than @c output_chunk_ms.
   *
   * @return Output chunks including the final partial chunk.
   */
  std::vector<OutputChunk> flush();

  /// @brief Clear all buffered audio and reset WebRTC and resampler state.
  void reset();

  /// @brief True if audio was dropped or WebRTC processing failed.
  bool had_error() const;

  /// @brief Clear the error flag and the stored WebRTC error code.
  void clear_error();

  /**
   * @brief WebRTC return code from the most recent processing failure.
   *
   * The value is 0 when there has been no failure since construction or since
   * the last clear_error() call. A negative value is the
   * @c webrtc::AudioProcessing::Error code reported by WebRTC.
   */
  int last_error() const;

  /// @brief Number of reference-timeout resets since construction.
  ///
  /// This diagnostic counter is not cleared by reset().
  uint64_t reference_reset_count() const;

  /// @brief Configured processing rate in Hz.
  int rate() const;

  /// @brief Number of samples in one 10 ms processing frame.
  int frame_size() const;

  /// @brief True if @p rate is supported by WebRTC AudioProcessing.
  static bool is_supported_rate(int rate);

  virtual ~AecProcessor() = default;

protected:
  /**
   * @brief Run the AEC backend on one 10 ms frame pair.
   *
   * This is a virtual seam: production uses the real WebRTC backend, while
   * tests can subclass AecProcessor to inject a WebRTC return code or a
   * non-finite output frame. The normal failure handling in
   * process_next_frame() is always exercised.
   *
   * @param reference Reference frame with frame_size() samples.
   * @param mic Microphone frame with frame_size() samples.
   * @param[out] processed Cleaned frame with frame_size() samples.
   * @param[out] error_code WebRTC return code when false is returned; 0 on
   *        success.
   * @return True on success, false if WebRTC reported an error.
   */
  virtual bool process_aec_frame(const float *reference, const float *mic,
                                 float *processed, int &error_code);

private:
  struct TimedBlock {
    std::vector<float> samples;
    int64_t stamp_ns = 0;
  };

  std::vector<float> pop_mic_frame(int64_t &stamp_ns);
  std::vector<float> pop_reference_frame();
  void process_next_frame(std::vector<OutputChunk> &output);
  void enforce_mic_capacity();
  void enforce_reference_capacity();
  int64_t next_mic_stamp() const;

  AecProcessorConfig config_;
  int frame_size_;
  int mic_buffer_frames_;
  int output_chunk_frames_;
  std::size_t mic_capacity_frames_;
  std::size_t reference_capacity_frames_;
  WebrtcAec aec_;
  audio_common_dsp::Resampler mic_resampler_;
  audio_common_dsp::Resampler reference_resampler_;
  std::deque<TimedBlock> mic_fifo_;
  std::deque<float> reference_fifo_;
  std::size_t mic_available_;
  std::size_t reference_available_;
  std::vector<float> output_samples_;
  int64_t output_stamp_ns_;
  bool had_error_;
  int last_error_;
  int64_t last_reference_stamp_ns_;
  bool reference_timed_out_;
  uint64_t reference_reset_count_;
};

} // namespace audio_aec

#endif // AUDIO_AEC__AEC_PROCESSOR_HPP_
