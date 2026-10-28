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

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <portaudio.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>

#include "audio_common_dsp/multi_channel_resampler.hpp"
#include "audio_common_msgs/msg/audio_data.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"

namespace audio_common {

/**
 * @brief ROS 2 node that subscribes to stamped audio messages and plays them
 *        through a PortAudio output device.
 *
 * One PortAudio stream is opened per unique combination of input sample format,
 * input sample rate, input channel count, output sample rate, and output
 * channel count. Mono/stereo channel conversion and sample-rate conversion are
 * performed automatically when the incoming audio differs from the configured
 * output format.
 *
 * @par ROS 2 Parameters
 * - `channels` (int, default 2): Number of output audio channels.
 * - `rate`     (int, default 0): Output sample rate. 0 selects the device
 *              default output sample rate.
 * - `device`   (int, default -1): PortAudio output device index.
 *              -1 selects the system default output device.
 * - `reference_topic` (string, default "played"): Topic for the actual
 *              resampled frames written to the device. An empty topic or
 *              `publish_reference=false` disables it.
 * - `publish_reference` (bool, default true): Publish the played frames.
 *
 * @par Subscriptions
 * - `audio` (audio_common_msgs/msg/AudioStamped, SensorDataQoS)
 *
 * @par Publications
 * - `played` (audio_common_msgs/msg/AudioStamped, SensorDataQoS) when enabled
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
  /// @brief A block of frames as they were actually sent to the device.
  struct ReferenceBlock {
    /// @brief Interleaved float samples written to the device.
    std::vector<float> samples;
    /// @brief Number of audio frames in @c samples, excluding channel
    /// multiplication.
    size_t frames{0};
    /// @brief Intended device render time of the first frame.
    builtin_interfaces::msg::Time stamp;
  };

  /// @brief State for one open PortAudio output stream and its AEC tap.
  struct PlaybackStream {
    /// @brief Open PortAudio output stream; null until successfully opened.
    PaStream *stream{nullptr};
    /// @brief Actual output sample rate negotiated with the device.
    int output_rate{0};
    /// @brief Number of interleaved output channels.
    int channels{1};
    /// @brief Node that owns this stream and receives reference callbacks.
    AudioPlayerNode *owner{nullptr};
    /// @brief Per-channel resampler for input at a different sample rate.
    audio_common_dsp::MultiChannelResampler resampler;
    /// @brief Timestamp (ns) of the last message written to this stream.
    int64_t last_stamp_ns = 0;

    /// @brief Protects @c queue.
    std::mutex queue_mutex;
    /// @brief Interleaved frames waiting to be pulled by the PortAudio
    /// callback.
    std::deque<float> queue;

    /// @brief Protects @c reference_queue and synchronizes the worker wait.
    std::mutex reference_mutex;
    /// @brief Wakes @c reference_worker when a new block is queued.
    std::condition_variable reference_cv;
    /// @brief Blocks written to the device, waiting to be published as the
    ///        AEC reference.
    std::deque<ReferenceBlock> reference_queue;
    /// @brief Thread that publishes @c reference_queue blocks.
    std::thread reference_worker;
    /// @brief Signals @c reference_worker to exit.
    std::atomic<bool> stop{false};
  };

  /// @brief ROS 2 subscription for incoming stamped audio messages.
  rclcpp::Subscription<audio_common_msgs::msg::AudioStamped>::SharedPtr
      audio_sub_;

  /// @brief Publisher for the actual output frames written to the device.
  rclcpp::Publisher<audio_common_msgs::msg::AudioStamped>::SharedPtr
      reference_pub_;

  /**
   * @brief Map from a stream-key string to the corresponding open PortAudio
   *        stream and resampler state.
   *
   * A new stream is created on demand the first time a particular input/output
   * format combination is encountered.
   */
  std::unordered_map<std::string, std::unique_ptr<PlaybackStream>> stream_dict_;

  /// @brief Number of output audio channels (ROS 2 parameter "channels").
  int channels_;

  /// @brief PortAudio output device index; -1 means the system default
  /// (ROS 2 parameter "device").
  int device_;

  /// @brief Output sample rate; 0 means the selected device's default rate
  /// (ROS 2 parameter "rate").
  int rate_;

  /**
   * @brief Subscription callback – opens a stream if needed, then writes the
   *        incoming audio data to the appropriate PortAudio stream.
   * @param msg Shared pointer to the received AudioStamped message.
   */
  void
  audio_callback(const audio_common_msgs::msg::AudioStamped::SharedPtr msg);

  /**
   * @brief Write a block of typed audio samples to a PortAudio stream,
   *        performing channel and sample-rate conversion when necessary.
   *
   * @param data        Input sample buffer received from the ROS 2 message.
   * @param format      PortAudio sample format constant of @p data.
   * @param channels    Channel count of the incoming @p data.
   * @param rate        Sample rate of the incoming @p data.
   * @param stamp_ns    Header timestamp of the message in nanoseconds (0 if
   *                    unset), used to detect stream discontinuities.
   * @param stream_key  Key used to look up the target stream in #stream_dict_.
   */
  void write_data(const audio_common_msgs::msg::AudioData &data, int format,
                  int channels, int rate, int64_t stamp_ns,
                  const std::string &stream_key);

  /**
   * @brief PortAudio output callback.
   *
   * Pulls interleaved frames from the stream queue, fills any underrun with
   * silence, and queues the exact output block for the AEC reference topic.
   *
   * @param input Unused input buffer.
   * @param[out] output Interleaved float output buffer to fill.
   * @param frames Number of frames requested by PortAudio.
   * @param time_info PortAudio timing information used for the reference
   *        timestamp.
   * @param status_flags PortAudio status flags (unused).
   * @param user_data Pointer to the owning PlaybackStream.
   * @return @c paContinue so PortAudio keeps the stream running.
   */
  static int playback_callback(const void *input, void *output,
                               unsigned long frames,
                               const PaStreamCallbackTimeInfo *time_info,
                               PaStreamCallbackFlags status_flags,
                               void *user_data);

  /**
   * @brief Publish blocks emitted by playback_callback().
   *
   * Waits on @c reference_cv, converts each queued block to an AudioStamped
   * message, and publishes it on the reference topic. Exits when the stream's
   * @c stop flag is set.
   *
   * @param stream Stream whose reference queue should be drained.
   */
  void reference_loop(PlaybackStream *stream);
};

} // namespace audio_common

#endif