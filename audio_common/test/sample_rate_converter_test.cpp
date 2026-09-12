#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "audio_common/sample_rate_converter.hpp"

namespace audio_common {

TEST(SampleRateConverterTest, RejectsInvalidArguments) {
  SampleRateConverter converter;

  EXPECT_THROW(converter.convert({1.0F}, 0, 44100, 48000),
               std::invalid_argument);
  EXPECT_THROW(converter.convert({1.0F}, 1, 0, 48000), std::invalid_argument);
  EXPECT_THROW(converter.convert({1.0F}, 1, 44100, 0), std::invalid_argument);
  EXPECT_THROW(converter.convert({1.0F, 2.0F}, 3, 44100, 48000),
               std::invalid_argument);
}

TEST(SampleRateConverterTest, FlushReturnsFinalFrameAndResetsState) {
  SampleRateConverter converter;

  EXPECT_EQ(converter.convert({1.0F, 2.0F}, 1, 44100, 44100),
            (std::vector<float>{1.0F}));
  EXPECT_EQ(converter.flush(), (std::vector<float>{2.0F}));
  EXPECT_TRUE(converter.flush().empty());
}

TEST(SampleRateConverterTest, PreservesChunkContinuity) {
  SampleRateConverter converter;

  const auto first = converter.convert({0.0F, 1.0F}, 1, 2, 4);
  const auto second = converter.convert({2.0F, 3.0F}, 1, 2, 4);
  const auto final = converter.flush();

  EXPECT_EQ(first, (std::vector<float>{0.0F, 0.5F}));
  EXPECT_EQ(second, (std::vector<float>{1.0F, 1.5F, 2.0F, 2.5F}));
  EXPECT_EQ(final, (std::vector<float>{3.0F}));
}

TEST(SampleRateConverterTest, ConvertsInterleavedChannels) {
  SampleRateConverter converter;

  const auto output = converter.convert({0.0F, 10.0F, 2.0F, 12.0F}, 2, 2, 1);
  const auto final = converter.flush();

  EXPECT_EQ(output, (std::vector<float>{0.0F, 10.0F}));
  EXPECT_EQ(final, (std::vector<float>{2.0F, 12.0F}));
}

} // namespace audio_common