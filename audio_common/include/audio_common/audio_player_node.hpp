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

#ifndef AUDIO_COMMON__AUDIO_PLAYER_NODE
#define AUDIO_COMMON__AUDIO_PLAYER_NODE

#include <memory>
#include <portaudio.h>
#include <rclcpp/rclcpp.hpp>

#include "audio_common_msgs/msg/audio_stamped.hpp"

namespace audio_common {

/**
 * @brief ROS 2 node that subscribes to stamped audio messages and plays them
 *        through a PortAudio output device.
 *
 * One PortAudio stream is opened per unique combination of sample format,
 * sample rate, and channel count.  Mono/stereo channel conversion is performed
 * automatically when the incoming channel count differs from the configured
 * output channel count.
 *
 * @par ROS 2 Parameters
 * - `channels` (int, default 2): Number of output audio channels.
 * - `device`   (int, default -1): PortAudio output device index.
 *              -1 selects the system default output device.
 *
 * @par Subscriptions
 * - `audio` (audio_common_msgs/msg/AudioStamped, SensorDataQoS)
 */
class AudioPlayerNode : public rclcpp::Node {
public:
  /**
   * @brief Construct the node, declare/read parameters and initialise
   *        PortAudio.
   * @throws std::runtime_error if PortAudio cannot be initialised.
   */
  AudioPlayerNode();

  /**
   * @brief Destructor – stops and closes all open PortAudio streams, then
   *        calls Pa_Terminate().
   */
  ~AudioPlayerNode() override;

private:
  /// @brief ROS 2 subscription for incoming stamped audio messages.
  rclcpp::Subscription<audio_common_msgs::msg::AudioStamped>::SharedPtr
      audio_sub_;

  /**
   * @brief Map from a stream-key string ("format_rate_channels") to the
   *        corresponding open PortAudio stream.
   *
   * A new stream is created on demand the first time a particular
   * format/rate/channel combination is encountered.
   */
  std::unordered_map<std::string, PaStream *> stream_dict_;

  /// @brief Number of output audio channels (ROS 2 parameter "channels").
  int channels_;

  /// @brief PortAudio output device index; -1 means the system default
  /// (ROS 2 parameter "device").
  int device_;

  /**
   * @brief Subscription callback – opens a stream if needed, then writes the
   *        incoming audio data to the appropriate PortAudio stream.
   * @param msg Shared pointer to the received AudioStamped message.
   */
  void
  audio_callback(const audio_common_msgs::msg::AudioStamped::SharedPtr msg);

  /**
   * @brief Write a block of typed audio samples to a PortAudio stream,
   *        performing mono↔stereo channel conversion when necessary.
   *
   * @tparam ContainerT Container type of the input sample buffer (e.g.
   *                    std::vector<int16_t>).
   * @param data        Input sample buffer received from the ROS 2 message.
   * @param channels    Channel count of the incoming @p data.
   * @param chunk       Number of frames in @p data.
   * @param stream_key  Key used to look up the target stream in #stream_dict_.
   */
  template <typename ContainerT>
  void write_data(const ContainerT &data, int channels, int chunk,
                  const std::string &stream_key);
};

} // namespace audio_common

#endif