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

#ifndef AUDIO_COMMON__WAVE_FILE_HPP
#define AUDIO_COMMON__WAVE_FILE_HPP

#include <fstream>
#include <string>
#include <vector>

namespace audio_common {

/**
 * @brief Lightweight reader for 16-bit PCM WAV files.
 *
 * Opens a WAV file, parses its header to extract audio metadata, and provides
 * sequential reading of sample data converted to normalised @c float values in
 * the range [-1.0, 1.0].  Only 16-bit PCM encoding is supported.
 *
 * @par Typical usage
 * @code
 * WaveFile wf("/path/to/file.wav");
 * if (!wf.open()) { /* handle error *\/ }
 * std::vector<float> buf;
 * while (wf.read(buf, 512)) {
 *     // process buf ...
 * }
 * @endcode
 */
class WaveFile {
public:
  /**
   * @brief Construct a WaveFile reader for the given path.
   *
   * The file is not opened until open() is called.
   *
   * @param filepath Absolute or relative path to the WAV file.
   */
  explicit WaveFile(const std::string &filepath);

  /**
   * @brief Destructor – closes the file stream if it is still open.
   */
  ~WaveFile();

  /**
   * @brief Open the WAV file and parse its header.
   *
   * Reads the RIFF/WAV header to populate #sample_rate_, #channels_, and
   * #bits_per_sample_, then seeks to the start of the PCM data section.
   *
   * @return @c true on success, @c false if the file could not be opened or
   *         the header is not a valid RIFF/WAV header.
   */
  bool open();

  /**
   * @brief Rewind to the beginning of the audio data.
   *
   * Closes and reopens the file, then calls open() so the read position is
   * reset to the first PCM sample.
   */
  void rewind();

  /**
   * @brief Read the next @p size frames from the file into @p buffer.
   *
   * Samples are converted from int16_t to @c float in the range [-1.0, 1.0]
   * using int16ToFloat().  @p buffer is resized to @p size × #channels_
   * elements.
   *
   * Only 16-bit PCM WAV files are supported; the call returns @c false
   * immediately for any other bit depth.
   *
   * @param[out] buffer Destination vector that receives the converted samples.
   * @param[in]  size   Number of frames (not bytes) to read.
   * @return @c true if exactly @p size frames were read, @c false on
   *         end-of-file, read error, or unsupported bit depth.
   */
  bool read(std::vector<float> &buffer, size_t size);

  /**
   * @brief Return the sample rate parsed from the WAV header.
   * @return Sample rate in Hz, or 0 if open() has not been called.
   */
  int get_sample_rate() const { return this->sample_rate_; }

  /**
   * @brief Return the number of audio channels parsed from the WAV header.
   * @return Channel count, or 0 if open() has not been called.
   */
  int get_num_channels() const { return this->channels_; }

  /**
   * @brief Return the bits-per-sample value parsed from the WAV header.
   * @return Bits per sample (e.g. 16), or 0 if open() has not been called.
   */
  int get_bits_per_sample() const { return this->bits_per_sample_; }

private:
  /// @brief Path to the WAV file supplied at construction.
  std::string filepath_;

  /// @brief File stream used for all read operations.
  std::ifstream file_;

  /// @brief Sample rate in Hz (populated by open()).
  int sample_rate_;

  /// @brief Number of audio channels (populated by open()).
  int channels_;

  /// @brief Bits per PCM sample (populated by open(); only 16 is supported).
  int bits_per_sample_;

  /**
   * @brief Convert a signed 16-bit PCM sample to a normalised float.
   *
   * @param sample Raw int16_t PCM value.
   * @return Floating-point value in the range [-1.0, 1.0].
   */
  float int16ToFloat(int16_t sample) { return sample / 32768.0f; }
};

} // namespace audio_common

#endif // WAVE_FILE_HPP
