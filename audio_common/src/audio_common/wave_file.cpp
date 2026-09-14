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

#include <cstdint>
#include <cstring>
#include <iostream>

#include "audio_common/wave_file.hpp"

using namespace audio_common;

namespace {

bool readUint16(std::ifstream &file, uint16_t &value) {
  file.read(reinterpret_cast<char *>(&value), sizeof(value));
  return file.gcount() == static_cast<std::streamsize>(sizeof(value));
}

bool readUint32(std::ifstream &file, uint32_t &value) {
  file.read(reinterpret_cast<char *>(&value), sizeof(value));
  return file.gcount() == static_cast<std::streamsize>(sizeof(value));
}

} // namespace

WaveFile::WaveFile(const std::string &filepath)
    : filepath_(filepath), sample_rate_(0), channels_(0), bits_per_sample_(0),
      data_offset_(0) {}

WaveFile::~WaveFile() { this->file_.close(); }

bool WaveFile::open() {
  this->file_.open(this->filepath_, std::ios::binary);
  if (!this->file_.is_open()) {
    std::cerr << "Failed to open file: " << this->filepath_ << std::endl;
    return false;
  }

  char riff_header[4];
  this->file_.read(riff_header, 4);
  if (this->file_.gcount() != 4 || std::strncmp(riff_header, "RIFF", 4) != 0) {
    std::cerr << "Invalid WAV file" << std::endl;
    return false;
  }

  this->file_.seekg(8);
  char wave_header[4];
  this->file_.read(wave_header, 4);
  if (this->file_.gcount() != 4 || std::strncmp(wave_header, "WAVE", 4) != 0) {
    std::cerr << "Invalid WAV file" << std::endl;
    return false;
  }

  bool found_fmt = false;
  bool found_data = false;

  // Walk the RIFF chunks to find the fmt and data chunks. Extra chunks (e.g.
  // LIST/INFO metadata) must be skipped instead of assuming a 44 byte header.
  while (!found_data) {
    char chunk_id[4];
    this->file_.read(chunk_id, 4);
    if (this->file_.gcount() != 4) {
      break;
    }

    uint32_t chunk_size = 0;
    if (!readUint32(this->file_, chunk_size)) {
      break;
    }

    if (std::strncmp(chunk_id, "fmt ", 4) == 0 && chunk_size >= 16) {
      uint16_t audio_format = 0;
      uint16_t channels = 0;
      uint32_t sample_rate = 0;
      uint16_t block_align = 0;
      uint16_t bits_per_sample = 0;

      if (!readUint16(this->file_, audio_format) ||
          !readUint16(this->file_, channels) ||
          !readUint32(this->file_, sample_rate) ||
          !this->file_.seekg(4, std::ios::cur) ||
          !readUint16(this->file_, block_align) ||
          !readUint16(this->file_, bits_per_sample)) {
        std::cerr << "Invalid WAV fmt chunk" << std::endl;
        return false;
      }

      if (audio_format != 1 || channels == 0 || sample_rate == 0 ||
          bits_per_sample != 16 || block_align < channels * 2) {
        std::cerr << "Unsupported WAV encoding (only 16-bit PCM is supported)"
                  << std::endl;
        return false;
      }

      this->channels_ = channels;
      this->sample_rate_ = static_cast<int>(sample_rate);
      this->bits_per_sample_ = bits_per_sample;
      found_fmt = true;

      const std::streamoff remaining =
          static_cast<std::streamoff>(chunk_size) - 16 +
          static_cast<std::streamoff>(chunk_size & 1U);
      if (remaining > 0) {
        this->file_.seekg(remaining, std::ios::cur);
      }

    } else if (std::strncmp(chunk_id, "data", 4) == 0) {
      this->data_offset_ = this->file_.tellg();
      found_data = true;

    } else {
      std::streamoff skip = chunk_size + (chunk_size & 1U);
      this->file_.seekg(skip, std::ios::cur);
    }

    if (!this->file_.good()) {
      break;
    }
  }

  if (!found_fmt || !found_data) {
    std::cerr << "WAV file is missing a valid fmt or data chunk" << std::endl;
    return false;
  }

  return true;
}

void WaveFile::rewind() {
  this->file_.clear();
  this->file_.seekg(this->data_offset_);
}

bool WaveFile::read(std::vector<float> &buffer, size_t size) {
  if (this->bits_per_sample_ != 16 || this->channels_ <= 0 || size == 0) {
    return false;
  }

  // Allocate temporary buffer to read int16 data
  std::vector<int16_t> temp_buffer(size * this->channels_);

  // Read raw int16 samples from the file
  this->file_.read(reinterpret_cast<char *>(temp_buffer.data()),
                   size * this->channels_ * sizeof(int16_t));

  const size_t samples_read =
      static_cast<size_t>(this->file_.gcount()) / sizeof(int16_t);
  const size_t frames_read = samples_read / this->channels_;
  if (frames_read == 0) {
    return false; // End of file or read error
  }

  // Convert int16 samples to float and store in buffer
  buffer.resize(frames_read * this->channels_);
  for (size_t i = 0; i < buffer.size(); ++i) {
    buffer[i] = int16ToFloat(temp_buffer[i]);
  }

  return true;
}