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

#ifndef AUDIO_COMMON__SAMPLE_RATE_CONVERTER
#define AUDIO_COMMON__SAMPLE_RATE_CONVERTER

#include <vector>

namespace audio_common {

class SampleRateConverter {
public:
  /**
   * @brief Convert interleaved audio samples to a different sample rate.
   *
   * The converter keeps one frame of look-ahead between calls. Call flush()
   * after the final input block to retrieve the retained final frame.
   *
   * @throws std::invalid_argument if a rate or channel count is not positive,
   *         or if the input does not contain complete frames.
   */
  std::vector<float> convert(const std::vector<float> &input_data, int channels,
                             int input_rate, int output_rate);

  /**
   * @brief Return the retained final frame and reset the converter state.
   */
  std::vector<float> flush();

private:
  double position_ = 0.0;
  std::vector<float> previous_frame_;
};

} // namespace audio_common

#endif