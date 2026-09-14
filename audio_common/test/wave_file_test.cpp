#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "audio_common/wave_file.hpp"

namespace audio_common {
namespace {

void writeBytes(std::ofstream &file, const void *data, size_t size) {
  file.write(reinterpret_cast<const char *>(data), size);
}

void writeUint16(std::ofstream &file, uint16_t value) {
  writeBytes(file, &value, sizeof(value));
}

void writeUint32(std::ofstream &file, uint32_t value) {
  writeBytes(file, &value, sizeof(value));
}

std::string writeTestWav(const std::vector<int16_t> &samples, uint16_t channels,
                         uint32_t sample_rate, uint16_t bits_per_sample,
                         bool add_extra_chunk) {
  const std::string path = "/tmp/audio_common_wave_file_test.wav";
  std::ofstream file(path, std::ios::binary | std::ios::trunc);

  const uint16_t bytes_per_sample = bits_per_sample / 8;
  const uint16_t block_align = channels * bytes_per_sample;
  const uint32_t data_size =
      static_cast<uint32_t>(samples.size() * sizeof(int16_t));
  const uint32_t extra_size = add_extra_chunk ? 26 : 0;
  const uint32_t riff_size =
      4 + (8 + 16) + (add_extra_chunk ? 8 + extra_size : 0) + 8 + data_size;

  writeBytes(file, "RIFF", 4);
  writeUint32(file, riff_size);
  writeBytes(file, "WAVE", 4);

  writeBytes(file, "fmt ", 4);
  writeUint32(file, 16);
  writeUint16(file, 1);
  writeUint16(file, channels);
  writeUint32(file, sample_rate);
  writeUint32(file, sample_rate * block_align);
  writeUint16(file, block_align);
  writeUint16(file, bits_per_sample);

  if (add_extra_chunk) {
    writeBytes(file, "LIST", 4);
    writeUint32(file, extra_size);
    writeBytes(file, "INFOISFT\x0e\x00\x00\x00Lavf58.29.10", extra_size);
  }

  writeBytes(file, "data", 4);
  writeUint32(file, data_size);
  writeBytes(file, samples.data(), data_size);
  file.close();

  return path;
}

class WaveFileTest : public ::testing::Test {
protected:
  void TearDown() override { std::remove(path_.c_str()); }

  std::string path_;
};

TEST_F(WaveFileTest, ReadsDataAfterExtraChunks) {
  const std::vector<int16_t> samples = {0,      16384, -16384, 32767,
                                        -32768, 100,   200,    300};
  this->path_ = writeTestWav(samples, 1, 44100, 16, true);

  WaveFile wav(this->path_);
  ASSERT_TRUE(wav.open());
  EXPECT_EQ(wav.get_sample_rate(), 44100);
  EXPECT_EQ(wav.get_num_channels(), 1);

  std::vector<float> buffer;
  ASSERT_TRUE(wav.read(buffer, samples.size()));
  ASSERT_EQ(buffer.size(), samples.size());
  for (size_t i = 0; i < samples.size(); ++i) {
    EXPECT_FLOAT_EQ(buffer[i], samples[i] / 32768.0F) << "sample " << i;
  }
}

TEST_F(WaveFileTest, ReturnsFinalPartialBlock) {
  const std::vector<int16_t> samples = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  this->path_ = writeTestWav(samples, 1, 16000, 16, false);

  WaveFile wav(this->path_);
  ASSERT_TRUE(wav.open());

  std::vector<float> buffer;
  ASSERT_TRUE(wav.read(buffer, 4));
  EXPECT_EQ(buffer.size(), 4U);
  ASSERT_TRUE(wav.read(buffer, 4));
  EXPECT_EQ(buffer.size(), 4U);
  ASSERT_TRUE(wav.read(buffer, 4));
  EXPECT_EQ(buffer.size(), 2U);
  EXPECT_FALSE(wav.read(buffer, 4));

  wav.rewind();
  ASSERT_TRUE(wav.read(buffer, 4));
  EXPECT_EQ(buffer.size(), 4U);
}

TEST_F(WaveFileTest, RejectsUnsupportedBitDepth) {
  const std::vector<int16_t> samples = {0, 1, 2, 3};
  this->path_ = writeTestWav(samples, 1, 16000, 8, false);

  WaveFile wav(this->path_);
  EXPECT_FALSE(wav.open());
}

} // namespace
} // namespace audio_common
