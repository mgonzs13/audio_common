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

#ifndef AUDIO_AEC__AEC_NODE_HPP_
#define AUDIO_AEC__AEC_NODE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "audio_aec/aec_processor.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"

namespace audio_aec {

/**
 * @brief ROS 2 node that removes the playback reference from microphone
 *        audio with WebRTC acoustic echo cancellation.
 *
 * The node subscribes to a microphone topic and a playback reference topic,
 * converts both to mono float32, and feeds them to AecProcessor. Cleaned
 * chunks are published at the configured output rate with the microphone
 * message's frame ID and the capture stamp of the chunk's first sample.
 *
 * @par ROS 2 Parameters
 * - `mic_topic` (string, default "audio"): Microphone input topic.
 * - `reference_topic` (string, default "audio_reference"): Playback reference
 *   topic.
 * - `output_topic` (string, default "audio_clean"): Cleaned output topic.
 * - `rate` (int, default 16000): Internal processing and output rate in Hz.
 * - `format` (int, default 8): PortAudio sample format constant for output.
 * - `mic_buffer_ms` (int, default 100): Microphone delay that lets reference
 *   audio arrive first; at most 500.
 * - `output_chunk_ms` (int, default 20): Duration of each output message.
 * - `stream_delay_ms` (int, default 0): Initial WebRTC delay hint.
 * - `reference_lead_ms` (int, default 0): Deliberate reference lead over the
 *   microphone timestamp, added to the WebRTC delay hint; at most 500.
 * - `reference_wait_ms` (int, default 100): How long a microphone frame may
 *   wait for its timestamped reference before the missing reference is
 *   treated as silence.
 * - `enable_high_pass` (bool, default true): Enable WebRTC's high-pass filter.
 * - `enable_noise_suppression` (bool, default false): Enable WebRTC noise
 *   suppression.
 * - `enable_agc` (bool, default false): Enable WebRTC automatic gain control.
 * - `reference_timeout_ms` (int, default 500): Silence interval after which
 *   the reference is considered gone.
 *
 * @par Subscriptions
 * - `audio` (audio_common_msgs/msg/AudioStamped, SensorDataQoS)
 * - `audio_reference` (audio_common_msgs/msg/AudioStamped, SensorDataQoS)
 *
 * @par Publications
 * - `audio_clean` (audio_common_msgs/msg/AudioStamped, SensorDataQoS)
 */
class AecNode : public rclcpp::Node {
public:
  /**
   * @brief Construct the node, declare and validate parameters, create the
   *        processor, and set up the subscriptions and publisher.
   *
   * @param options Node options.
   * @throws std::runtime_error if a parameter is invalid.
   */
  explicit AecNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  /// @brief Echo-cancelling audio processor shared by both callbacks.
  std::unique_ptr<AecProcessor> processor_;

  /// @brief Subscription to microphone audio.
  rclcpp::Subscription<audio_common_msgs::msg::AudioStamped>::SharedPtr
      mic_sub_;

  /// @brief Subscription to the playback reference audio.
  rclcpp::Subscription<audio_common_msgs::msg::AudioStamped>::SharedPtr
      reference_sub_;

  /// @brief Publisher for cleaned audio.
  rclcpp::Publisher<audio_common_msgs::msg::AudioStamped>::SharedPtr
      output_pub_;

  /// @brief Serializes access to the processor and the latest frame ID.
  std::mutex mutex_;

  /// @brief Frame ID of the most recent microphone message.
  std::string latest_frame_id_;

  /// @brief PortAudio sample format constant used for output.
  int format_;

  /// @brief Last reference-timeout reset reported by the processor.
  uint64_t last_reference_reset_count_;

  /**
   * @brief Convert and enqueue microphone audio, then publish ready chunks.
   *
   * Invalid messages are logged and ignored.
   *
   * @param msg Microphone audio message.
   */
  void mic_callback(const audio_common_msgs::msg::AudioStamped::SharedPtr msg);

  /**
   * @brief Convert and enqueue playback reference audio, then publish ready
   *        chunks.
   *
   * Invalid messages are logged and ignored.
   *
   * @param msg Reference audio message.
   */
  void
  reference_callback(const audio_common_msgs::msg::AudioStamped::SharedPtr msg);

  /**
   * @brief Publish every output chunk currently available.
   *
   * The caller must hold mutex_.
   */
  void publish_ready();
};

} // namespace audio_aec

#endif // AUDIO_AEC__AEC_NODE_HPP_
