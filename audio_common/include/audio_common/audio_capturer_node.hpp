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

#ifndef AUDIO_COMMON__AUDIO_CAPTURER_NODE
#define AUDIO_COMMON__AUDIO_CAPTURER_NODE

#include <memory>
#include <portaudio.h>
#include <rclcpp/rclcpp.hpp>

#include "audio_common_msgs/msg/audio_stamped.hpp"

namespace audio_common {

/**
 * @brief ROS 2 node that captures audio from a microphone via PortAudio and
 *        publishes it as stamped audio messages.
 *
 * The node opens a PortAudio input stream in the constructor and then
 * publishes captured frames continuously inside work().
 *
 * @par ROS 2 Parameters
 * - `format`   (int, default paInt16): PortAudio sample format constant.
 * - `channels` (int, default 1): Number of input audio channels.
 * - `rate`     (int, default 16000): Sample rate in Hz.
 * - `chunk`    (int, default 512): Number of frames per read operation.
 * - `device`   (int, default -1): PortAudio input device index.
 *              -1 selects the system default input device.
 * - `frame_id` (string, default ""): TF frame ID attached to published
 *              message headers.
 *
 * @par Publications
 * - `audio` (audio_common_msgs/msg/AudioStamped, SensorDataQoS)
 */
class AudioCapturerNode : public rclcpp::Node {
public:
  /**
   * @brief Construct the node, declare/read parameters, open and start the
   *        PortAudio input stream, and create the publisher.
   * @throws std::runtime_error if PortAudio initialisation or stream
   *         open/start fails.
   */
  AudioCapturerNode();

  /**
   * @brief Destructor – stops and closes the PortAudio input stream, then
   *        calls Pa_Terminate().
   */
  ~AudioCapturerNode() override;

  /**
   * @brief Blocking capture loop – reads audio chunks from PortAudio and
   *        publishes them until `rclcpp::ok()` returns false.
   *
   * This method should be called from the node's main thread after
   * construction.
   */
  void work();

private:
  /// @brief Active PortAudio input stream handle.
  PaStream *stream_;

  /// @brief PortAudio sample format (e.g. paInt16, paFloat32).
  /// Corresponds to the ROS 2 parameter \"format\".
  int format_;

  /// @brief Number of input audio channels.
  /// Corresponds to the ROS 2 parameter \"channels\".
  int channels_;

  /// @brief Sample rate in Hz.
  /// Corresponds to the ROS 2 parameter \"rate\".
  int rate_;

  /// @brief Number of frames captured per read call.
  /// Corresponds to the ROS 2 parameter \"chunk\".
  int chunk_;

  /// @brief TF frame ID used in published message headers.
  /// Corresponds to the ROS 2 parameter \"frame_id\".
  std::string frame_id_;

  /// @brief Publisher for captured audio data.
  rclcpp::Publisher<audio_common_msgs::msg::AudioStamped>::SharedPtr audio_pub_;

  /**
   * @brief Read one chunk of audio samples from the PortAudio stream.
   *
   * @tparam T Sample type that matches the configured PortAudio format
   *           (e.g. @c float, @c int16_t, @c int8_t, @c uint8_t).
   * @return Vector of @c chunk_ × @c channels_ samples.
   */
  template <typename T> std::vector<T> read_data();
};

} // namespace audio_common

#endif