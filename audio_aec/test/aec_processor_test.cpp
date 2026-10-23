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

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

#include "audio_aec/aec_processor.hpp"

namespace {

std::vector<float> sine(double frequency, int rate, double seconds,
                        float amplitude) {
  const size_t frames = static_cast<size_t>(rate * seconds);
  std::vector<float> data(frames);
  for (size_t i = 0; i < frames; ++i) {
    data[i] = amplitude *
              static_cast<float>(std::sin(2.0 * M_PI * frequency * i / rate));
  }
  return data;
}

double rms(const std::vector<float> &data) {
  double sum = 0.0;
  for (float sample : data) {
    sum += static_cast<double>(sample) * sample;
  }
  return data.empty() ? 0.0 : std::sqrt(sum / data.size());
}

void scale_to_rms(std::vector<float> &data, double target_rms) {
  const double current = rms(data);
  if (current <= 0.0) {
    return;
  }
  const double scale = target_rms / current;
  for (float &sample : data) {
    sample = static_cast<float>(sample * scale);
  }
}

void construct_with_mic_buffer(int mic_buffer_ms) {
  audio_aec::AecProcessorConfig config;
  config.rate = 16000;
  config.mic_buffer_ms = mic_buffer_ms;
  audio_aec::AecProcessor processor(config);
}

/// @brief Deterministic speech-like chirp with a syllabic envelope.
std::vector<float> speech_like_reference(int rate, double seconds,
                                         double start_hz, double end_hz,
                                         double phase_offset) {
  const size_t frames = static_cast<size_t>(rate * seconds);
  std::vector<float> data(frames);
  const double sweep = (end_hz - start_hz) / seconds;

  for (size_t i = 0; i < frames; ++i) {
    const double t = static_cast<double>(i) / rate;
    const double phase =
        2.0 * M_PI * (start_hz * t + 0.5 * sweep * t * t) + phase_offset;
    const double envelope = 0.65 + 0.35 * std::sin(2.0 * M_PI * 2.9 * t + 0.2);
    data[i] = static_cast<float>(envelope * std::sin(phase));
  }

  scale_to_rms(data, 0.15);
  return data;
}

/// @brief Deterministic speech-like multitone with a syllabic envelope.
///
/// The carriers are unrelated to the reference chirp, so this signal exercises
/// double talk rather than more echo.
std::vector<float> speech_like_near_end(int rate, double seconds) {
  const double carriers[] = {330.0,  550.0,  770.0,  1100.0,
                             1430.0, 1870.0, 2310.0, 2750.0};
  const size_t carrier_count = sizeof(carriers) / sizeof(carriers[0]);
  const size_t frames = static_cast<size_t>(rate * seconds);
  std::vector<float> data(frames);

  for (size_t i = 0; i < frames; ++i) {
    const double t = static_cast<double>(i) / rate;
    const double envelope = 0.55 + 0.45 * std::sin(2.0 * M_PI * 3.7 * t + 0.8) *
                                       std::sin(2.0 * M_PI * 1.369 * t + 1.1);
    double sum = 0.0;
    for (size_t k = 0; k < carrier_count; ++k) {
      sum += std::sin(2.0 * M_PI * carriers[k] * t + 0.7 * k + 0.8);
    }
    data[i] =
        static_cast<float>(envelope * sum / static_cast<double>(carrier_count));
  }

  // Same RMS as a 0.2-amplitude sine, matching the previous near-end level.
  scale_to_rms(data, 0.14);
  return data;
}

/// @brief Best normalized cross-correlation over +/- @p max_lag_samples.
///
/// Both signals are decimated by four before correlating to keep the search
/// cheap; this is a sanity check, not a precise delay estimate.
double best_lag_correlation(const std::vector<float> &signal,
                            const std::vector<float> &reference,
                            int max_lag_samples) {
  const size_t length = std::min(signal.size(), reference.size());
  const size_t stride = 4;
  const int lag_limit = max_lag_samples / static_cast<int>(stride);
  double best = -1.0;

  for (int lag = -lag_limit; lag <= lag_limit; ++lag) {
    double product = 0.0;
    double signal_energy = 0.0;
    double reference_energy = 0.0;

    for (size_t i = 0; i < length; i += stride) {
      const long shifted = static_cast<long>(i) + static_cast<long>(lag) * 4;
      if (shifted < 0 || shifted >= static_cast<long>(length)) {
        continue;
      }
      const double a = signal[i];
      const double b = reference[static_cast<size_t>(shifted)];
      product += a * b;
      signal_energy += a * a;
      reference_energy += b * b;
    }

    if (signal_energy > 0.0 && reference_energy > 0.0) {
      best =
          std::max(best, product / std::sqrt(signal_energy * reference_energy));
    }
  }

  return best;
}

int64_t sample_stamp(int64_t start_ns, int sample, int rate) {
  return start_ns + static_cast<int64_t>(sample) * 1000000000LL / rate;
}

/**
 * @brief AecProcessor with a scriptable AEC backend for failure-path tests.
 *
 * Only the backend result is injected; the production failure handling in
 * AecProcessor (fallback sanitization, reset, had_error(), last_error()) is
 * the code under test.
 */
class ScriptedAecProcessor : public audio_aec::AecProcessor {
public:
  using audio_aec::AecProcessor::AecProcessor;

  void fail_next(int error_code) { this->failure_code_ = error_code; }
  void emit_non_finite_next() { this->non_finite_next_ = true; }

protected:
  bool process_aec_frame(const float *reference, const float *mic,
                         float *processed, int &error_code) override {
    if (this->failure_code_ != 0) {
      error_code = this->failure_code_;
      this->failure_code_ = 0;
      return false;
    }
    if (this->non_finite_next_) {
      this->non_finite_next_ = false;
      for (int i = 0; i < this->frame_size(); ++i) {
        processed[i] = std::numeric_limits<float>::quiet_NaN();
      }
      return true;
    }
    return audio_aec::AecProcessor::process_aec_frame(reference, mic, processed,
                                                      error_code);
  }

private:
  int failure_code_ = 0;
  bool non_finite_next_ = false;
};

} // namespace

TEST(AecProcessorTest, NoReferencePassesMicThrough) {
  audio_aec::AecProcessorConfig config;
  config.rate = 16000;
  config.mic_buffer_ms = 100;
  config.output_chunk_ms = 20;
  audio_aec::AecProcessor processor(config);

  ASSERT_EQ(processor.rate(), 16000);
  ASSERT_EQ(processor.frame_size(), 160);

  const auto input = sine(440.0, 16000, 0.5, 0.5);
  std::vector<audio_aec::OutputChunk> chunks;
  const int chunk_frames = 320;
  for (size_t offset = 0; offset < input.size(); offset += chunk_frames) {
    const size_t count = std::min<size_t>(chunk_frames, input.size() - offset);
    std::vector<float> block(input.begin() + offset,
                             input.begin() + offset + count);
    processor.push_mic(block, 16000, sample_stamp(1000000000LL, offset, 16000));
    auto ready = processor.pop_output();
    chunks.insert(chunks.end(), ready.begin(), ready.end());
  }
  auto tail = processor.flush();
  chunks.insert(chunks.end(), tail.begin(), tail.end());

  std::vector<float> output;
  for (const auto &chunk : chunks) {
    output.insert(output.end(), chunk.samples.begin(), chunk.samples.end());
  }

  ASSERT_NEAR(output.size(), input.size(), 2u * processor.frame_size());
  EXPECT_GT(rms(output), 0.1);
  EXPECT_LT(rms(output), 0.6);
  EXPECT_EQ(chunks.front().stamp_ns, 1000000000LL);
  for (size_t i = 0; i + 1 < chunks.size(); ++i) {
    EXPECT_EQ(chunks[i].samples.size(), static_cast<size_t>(16000 * 20 / 1000));
  }
  EXPECT_LE(chunks.back().samples.size(),
            static_cast<size_t>(16000 * 20 / 1000));
}

TEST(AecProcessorTest, RejectsMicBufferAboveMaximum) {
  EXPECT_THROW(construct_with_mic_buffer(510), std::invalid_argument);
  EXPECT_THROW(construct_with_mic_buffer(1000), std::invalid_argument);
}

TEST(AecProcessorTest, StreamsAtMaximumMicBuffer) {
  audio_aec::AecProcessorConfig config;
  config.rate = 16000;
  config.mic_buffer_ms = 500;
  config.output_chunk_ms = 20;
  audio_aec::AecProcessor processor(config);

  const auto input = sine(440.0, 16000, 5.0, 0.4);
  std::vector<audio_aec::OutputChunk> chunks;
  const size_t block_frames = 1600; // 100 ms at 16 kHz.
  for (size_t offset = 0; offset < input.size(); offset += block_frames) {
    const size_t count = std::min(block_frames, input.size() - offset);
    const std::vector<float> block(input.begin() + offset,
                                   input.begin() + offset + count);
    processor.push_mic(block, 16000, sample_stamp(1000000000LL, offset, 16000));
    auto ready = processor.pop_output();
    chunks.insert(chunks.end(), ready.begin(), ready.end());
  }
  const auto tail = processor.flush();
  chunks.insert(chunks.end(), tail.begin(), tail.end());

  size_t total = 0;
  for (const auto &chunk : chunks) {
    total += chunk.samples.size();
  }

  EXPECT_NEAR(static_cast<double>(total), static_cast<double>(input.size()),
              static_cast<double>(processor.frame_size()));
  EXPECT_FALSE(processor.had_error());
}

TEST(AecProcessorTest, SyntheticEchoIsReduced) {
  audio_aec::AecProcessorConfig config;
  config.rate = 16000;
  config.mic_buffer_ms = 100;
  config.output_chunk_ms = 20;
  audio_aec::AecProcessor processor(config);

  const int rate = 16000;
  const int delay_samples = rate * 20 / 1000;
  const size_t total = rate * 4;
  const std::vector<float> reference =
      speech_like_reference(rate, 4.0, 200.0, 3000.0, 0.0);
  ASSERT_EQ(reference.size(), total);
  std::vector<float> mic(total, 0.0f);
  for (size_t i = delay_samples; i < total; ++i) {
    mic[i] = 0.5f * reference[i - delay_samples];
  }

  std::vector<audio_aec::OutputChunk> chunks;
  for (size_t offset = 0; offset < total; offset += 160) {
    const std::vector<float> ref_block(reference.begin() + offset,
                                       reference.begin() + offset + 160);
    const std::vector<float> mic_block(mic.begin() + offset,
                                       mic.begin() + offset + 160);
    const int64_t stamp = sample_stamp(1000000000LL, offset, rate);
    processor.push_reference(ref_block, rate, stamp);
    processor.push_mic(mic_block, rate, stamp);
    auto ready = processor.pop_output();
    chunks.insert(chunks.end(), ready.begin(), ready.end());
  }
  auto tail = processor.flush();
  chunks.insert(chunks.end(), tail.begin(), tail.end());

  std::vector<float> output;
  for (const auto &chunk : chunks) {
    output.insert(output.end(), chunk.samples.begin(), chunk.samples.end());
  }
  ASSERT_GE(output.size(), static_cast<size_t>(rate));
  const size_t window = static_cast<size_t>(rate);
  const std::vector<float> mic_tail(mic.end() - window, mic.end());
  const std::vector<float> out_tail(output.end() - window, output.end());
  const double erle = 20.0 * std::log10(rms(mic_tail) / rms(out_tail));
  EXPECT_GT(erle, 10.0);
  EXPECT_FALSE(processor.had_error());
}

TEST(AecProcessorTest, DoubleTalkPreservesNearEnd) {
  audio_aec::AecProcessorConfig config;
  config.rate = 16000;
  config.mic_buffer_ms = 100;
  config.output_chunk_ms = 20;
  audio_aec::AecProcessor processor(config);

  const int rate = 16000;
  const int delay_samples = rate * 20 / 1000;
  const size_t total = rate * 4;
  const std::vector<float> reference =
      speech_like_reference(rate, 4.0, 200.0, 3000.0, 0.7);
  ASSERT_EQ(reference.size(), total);
  std::vector<float> mic(total, 0.0f);
  for (size_t i = delay_samples; i < total; ++i) {
    mic[i] = 0.5f * reference[i - delay_samples];
  }
  const std::vector<float> near_end = speech_like_near_end(rate, 2.0);
  for (size_t i = 0; i < near_end.size(); ++i) {
    mic[total - near_end.size() + i] += near_end[i];
  }

  std::vector<audio_aec::OutputChunk> chunks;
  for (size_t offset = 0; offset < total; offset += 160) {
    const std::vector<float> ref_block(reference.begin() + offset,
                                       reference.begin() + offset + 160);
    const std::vector<float> mic_block(mic.begin() + offset,
                                       mic.begin() + offset + 160);
    const int64_t stamp = sample_stamp(1000000000LL, offset, rate);
    processor.push_reference(ref_block, rate, stamp);
    processor.push_mic(mic_block, rate, stamp);
    auto ready = processor.pop_output();
    chunks.insert(chunks.end(), ready.begin(), ready.end());
  }
  auto tail = processor.flush();
  chunks.insert(chunks.end(), tail.begin(), tail.end());

  std::vector<float> output;
  for (const auto &chunk : chunks) {
    output.insert(output.end(), chunk.samples.begin(), chunk.samples.end());
  }
  const size_t window = static_cast<size_t>(rate);
  ASSERT_GE(output.size(), window);
  const std::vector<float> out_tail(output.end() - window, output.end());
  const std::vector<float> near_tail(near_end.end() - window, near_end.end());
  const double out_rms = rms(out_tail);
  const double near_rms = rms(near_tail);
  EXPECT_GT(out_rms, 0.4 * near_rms);
  EXPECT_LT(out_rms, 1.5 * near_rms);
  EXPECT_GT(best_lag_correlation(out_tail, near_tail, rate * 200 / 1000), 0.3);
  EXPECT_FALSE(processor.had_error());
}

TEST(AecProcessorTest, ReferenceGapRecovers) {
  audio_aec::AecProcessorConfig config;
  config.rate = 16000;
  config.mic_buffer_ms = 100;
  config.output_chunk_ms = 20;
  config.reference_timeout_ms = 500;
  audio_aec::AecProcessor processor(config);

  const int rate = 16000;
  const int delay_samples = rate * 20 / 1000;
  const size_t total = rate * 8;
  std::vector<float> reference =
      speech_like_reference(rate, 8.0, 200.0, 3000.0, 0.3);
  ASSERT_EQ(reference.size(), total);
  for (size_t i = rate * 2; i < rate * 4; ++i) {
    reference[i] = 0.0f;
  }
  std::vector<float> mic(total, 0.0f);
  for (size_t i = delay_samples; i < total; ++i) {
    mic[i] = 0.5f * reference[i - delay_samples];
  }

  std::vector<audio_aec::OutputChunk> chunks;
  for (size_t offset = 0; offset < total; offset += 160) {
    const std::vector<float> ref_block(reference.begin() + offset,
                                       reference.begin() + offset + 160);
    const std::vector<float> mic_block(mic.begin() + offset,
                                       mic.begin() + offset + 160);
    const int64_t stamp = sample_stamp(1000000000LL, offset, rate);
    if (offset < rate * 2 || offset >= rate * 4) {
      processor.push_reference(ref_block, rate, stamp);
    }
    processor.push_mic(mic_block, rate, stamp);
    auto ready = processor.pop_output();
    chunks.insert(chunks.end(), ready.begin(), ready.end());
  }
  auto tail = processor.flush();
  chunks.insert(chunks.end(), tail.begin(), tail.end());

  std::vector<float> output;
  for (const auto &chunk : chunks) {
    output.insert(output.end(), chunk.samples.begin(), chunk.samples.end());
  }
  ASSERT_GE(output.size(), static_cast<size_t>(rate * 2));
  const size_t window = static_cast<size_t>(rate * 2);
  const std::vector<float> mic_tail(mic.end() - window, mic.end());
  const std::vector<float> out_tail(output.end() - window, output.end());
  const double erle = 20.0 * std::log10(rms(mic_tail) / rms(out_tail));
  EXPECT_GT(erle, 5.0);
  EXPECT_EQ(processor.reference_reset_count(), 1u);
  EXPECT_FALSE(processor.had_error());
}

TEST(AecProcessorTest, NonFiniteInputAt48kDoesNotPoisonResampler) {
  audio_aec::AecProcessorConfig config;
  config.rate = 16000;
  config.mic_buffer_ms = 100;
  config.output_chunk_ms = 20;
  audio_aec::AecProcessor processor(config);

  const int input_rate = 48000;
  std::vector<float> bad(4800, 0.0f); // 100 ms.
  bad[100] = std::numeric_limits<float>::quiet_NaN();
  bad[200] = std::numeric_limits<float>::infinity();
  bad[300] = -std::numeric_limits<float>::infinity();
  processor.push_mic(bad, input_rate, 1000000000LL);
  processor.push_reference(bad, input_rate, 1000000000LL);

  std::vector<float> output;
  {
    auto ready = processor.pop_output();
    for (const auto &chunk : ready) {
      output.insert(output.end(), chunk.samples.begin(), chunk.samples.end());
    }
  }
  processor.clear_error();

  const auto tone = sine(440.0, input_rate, 2.0, 0.5);
  const size_t block_frames = 4800; // 100 ms at 48 kHz.
  for (size_t offset = 0; offset < tone.size(); offset += block_frames) {
    const size_t count = std::min(block_frames, tone.size() - offset);
    const std::vector<float> block(tone.begin() + offset,
                                   tone.begin() + offset + count);
    processor.push_mic(block, input_rate,
                       sample_stamp(1000000000LL, offset, input_rate));
    auto ready = processor.pop_output();
    for (const auto &chunk : ready) {
      output.insert(output.end(), chunk.samples.begin(), chunk.samples.end());
    }
  }
  const auto tail = processor.flush();
  for (const auto &chunk : tail) {
    output.insert(output.end(), chunk.samples.begin(), chunk.samples.end());
  }

  for (float sample : output) {
    EXPECT_TRUE(std::isfinite(sample));
  }
  ASSERT_GE(output.size(), static_cast<size_t>(16000));
  const std::vector<float> out_tail(output.end() - 16000, output.end());
  EXPECT_GT(rms(out_tail), 0.1);
  EXPECT_FALSE(processor.had_error());
}

TEST(AecProcessorTest, WebrtcFailureFallsBackResetsAndReportsCode) {
  audio_aec::AecProcessorConfig config;
  config.rate = 16000;
  config.mic_buffer_ms = 0;
  config.output_chunk_ms = 10;
  ScriptedAecProcessor processor(config);

  processor.fail_next(-6);
  const std::vector<float> mic(160, 0.25f);
  processor.push_mic(mic, 16000, 1000000000LL);
  const auto chunks = processor.pop_output();
  ASSERT_EQ(chunks.size(), 1u);
  EXPECT_EQ(chunks[0].samples, mic);
  EXPECT_TRUE(processor.had_error());
  EXPECT_EQ(processor.last_error(), -6);

  processor.clear_error();
  EXPECT_FALSE(processor.had_error());
  EXPECT_EQ(processor.last_error(), 0);

  std::vector<float> output;
  const auto tone = sine(440.0, 16000, 0.2, 0.5);
  for (size_t offset = 0; offset < tone.size(); offset += 160) {
    const std::vector<float> block(tone.begin() + offset,
                                   tone.begin() + offset + 160);
    processor.push_mic(block, 16000, sample_stamp(1010000000LL, offset, 16000));
    auto ready = processor.pop_output();
    for (const auto &chunk : ready) {
      output.insert(output.end(), chunk.samples.begin(), chunk.samples.end());
    }
  }
  const auto tail = processor.flush();
  for (const auto &chunk : tail) {
    output.insert(output.end(), chunk.samples.begin(), chunk.samples.end());
  }

  EXPECT_FALSE(processor.had_error());
  EXPECT_EQ(processor.last_error(), 0);
  EXPECT_GT(rms(output), 0.1);
  for (float sample : output) {
    EXPECT_TRUE(std::isfinite(sample));
  }
}

TEST(AecProcessorTest, NonFiniteWebrtcOutputIsSanitizedAndResets) {
  audio_aec::AecProcessorConfig config;
  config.rate = 16000;
  config.mic_buffer_ms = 0;
  config.output_chunk_ms = 10;
  ScriptedAecProcessor processor(config);

  processor.emit_non_finite_next();
  const std::vector<float> mic(160, 0.5f);
  processor.push_mic(mic, 16000, 1000000000LL);
  const auto chunks = processor.pop_output();
  ASSERT_EQ(chunks.size(), 1u);
  for (float sample : chunks[0].samples) {
    EXPECT_EQ(sample, 0.0f);
  }
  EXPECT_TRUE(processor.had_error());
  EXPECT_EQ(processor.last_error(), 0);

  processor.clear_error();
  std::vector<float> output;
  const auto tone = sine(440.0, 16000, 0.2, 0.5);
  for (size_t offset = 0; offset < tone.size(); offset += 160) {
    const std::vector<float> block(tone.begin() + offset,
                                   tone.begin() + offset + 160);
    processor.push_mic(block, 16000, sample_stamp(1010000000LL, offset, 16000));
    auto ready = processor.pop_output();
    for (const auto &chunk : ready) {
      output.insert(output.end(), chunk.samples.begin(), chunk.samples.end());
    }
  }
  const auto tail = processor.flush();
  for (const auto &chunk : tail) {
    output.insert(output.end(), chunk.samples.begin(), chunk.samples.end());
  }

  EXPECT_FALSE(processor.had_error());
  EXPECT_GT(rms(output), 0.1);
  for (float sample : output) {
    EXPECT_TRUE(std::isfinite(sample));
  }
}

TEST(AecProcessorTest, ReferenceTimeoutClearsPrefilledFifo) {
  audio_aec::AecProcessorConfig config;
  config.rate = 16000;
  config.mic_buffer_ms = 0;
  config.output_chunk_ms = 10;
  config.reference_timeout_ms = 100;
  audio_aec::AecProcessor processor(config);

  const auto reference = sine(440.0, 16000, 1.0, 0.3);
  processor.push_reference(reference, 16000, 1000000000LL);

  const auto mic = sine(440.0, 16000, 0.01, 0.5);
  processor.push_mic(mic, 16000, 2000000000LL);

  EXPECT_EQ(processor.reference_reset_count(), 1u);
  EXPECT_FALSE(processor.had_error());

  const auto chunks = processor.pop_output();
  ASSERT_EQ(chunks.size(), 1u);
  EXPECT_GT(rms(chunks[0].samples), 0.1);
}
