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

#ifndef AUDIO_AEC__RESAMPLER_HPP_
#define AUDIO_AEC__RESAMPLER_HPP_

#include <vector>

namespace audio_aec {

/**
 * @brief Streaming sample-rate converter with an anti-alias low-pass filter.
 *
 * Downsampling is preceded by a cascade of two biquad low-pass filters that
 * implement a 4th-order Butterworth response at 0.45 * output_rate. Linear
 * interpolation then produces the output samples. Filter and interpolation
 * state are preserved across calls, so splitting the input into blocks gives
 * the same result as processing it in one call.
 */
class Resampler {
public:
  Resampler();

  /**
   * @brief Convert one block of mono float samples.
   *
   * @param input Mono float samples.
   * @param input_rate Sample rate of @p input in Hz; must be positive.
   * @param output_rate Desired sample rate in Hz; must be positive.
   * @return Converted samples.
   * @throws std::invalid_argument if either sample rate is not positive.
   */
  std::vector<float> process(const std::vector<float> &input, int input_rate,
                             int output_rate);

  /**
   * @brief Drain the remaining resampled output samples and reset.
   *
   * All returned samples are at the output rate, regardless of the input to
   * output rate ratio. When the last buffered input sample has no
   * interpolation partner, it is held for the final output samples.
   *
   * @return Remaining output-rate samples, then all state is cleared.
   */
  std::vector<float> flush();

  /// @brief Clear buffered samples, interpolation phase, and filter state.
  void reset();

private:
  struct Biquad {
    double b0 = 1.0;
    double b1 = 0.0;
    double b2 = 0.0;
    double a1 = 0.0;
    double a2 = 0.0;
    double z1 = 0.0;
    double z2 = 0.0;
  };

  static Biquad make_lowpass(double cutoff_hz, double sample_rate, double q);
  static float apply_biquad(Biquad &filter, float sample);

  std::vector<float> buffer_;
  double position_;
  int input_rate_;
  int output_rate_;
  bool filter_enabled_;
  Biquad lowpass1_;
  Biquad lowpass2_;
};

} // namespace audio_aec

#endif // AUDIO_AEC__RESAMPLER_HPP_
