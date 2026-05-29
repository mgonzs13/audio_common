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

#ifndef AUDIO_COMMON__MUSIC_NODE
#define AUDIO_COMMON__MUSIC_NODE

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <audio_common_msgs/msg/audio_stamped.hpp>
#include <audio_common_msgs/srv/music_play.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace audio_common {

/**
 * @brief ROS 2 node that streams WAV music files by publishing audio chunks
 *        and exposes ROS 2 services to control playback.
 *
 * Playback runs in a dedicated background thread (@c publish_thread_).  Audio
 * data is read from a WAV file using WaveFile and published as
 * @c AudioStamped messages at a rate determined by the file's sample rate and
 * the chunk size.  The node supports optional looping as well as pause/resume
 * without interrupting the publishing thread.
 *
 * @par ROS 2 Parameters
 * - `chunk`    (int, default 2048): Number of float samples per published
 *              message.
 * - `frame_id` (string, default ""): TF frame ID attached to published
 *              message headers.
 *
 * @par Publications
 * - `audio` (audio_common_msgs/msg/AudioStamped, SensorDataQoS)
 *
 * @par Services
 * - `music_play`   (audio_common_msgs/srv/MusicPlay)
 * - `music_stop`   (std_srvs/srv/Trigger)
 * - `music_pause`  (std_srvs/srv/Trigger)
 * - `music_resume` (std_srvs/srv/Trigger)
 */
class MusicNode : public rclcpp::Node {
public:
  /**
   * @brief Construct the node, declare/read parameters, create the publisher
   *        and all service servers.
   */
  MusicNode();

  /**
   * @brief Destructor – signals the publishing thread to stop and joins it.
   */
  ~MusicNode() override;

private:
  /// @brief Number of float samples per published message.
  /// Corresponds to the ROS 2 parameter \"chunk\".
  int chunk_;

  /// @brief TF frame ID used in published message headers.
  /// Corresponds to the ROS 2 parameter \"frame_id\".
  std::string frame_id_;

  /// @brief When @c true the publishing thread suspends at the next chunk
  /// boundary.
  std::atomic<bool> pause_music_;

  /// @brief When @c true the publishing thread exits its loop and terminates.
  std::atomic<bool> stop_music_;

  /// @brief When @c true the track restarts automatically after reaching the
  /// end.
  bool audio_loop_;

  /// @brief @c true while the publishing thread is actively streaming audio.
  std::atomic<bool> is_thread_alive_;

  /// @brief Background thread that reads the WAV file and publishes audio
  /// chunks.
  std::thread publish_thread_;

  /// @brief Mutex used together with #pause_cv_ to block the publishing thread.
  std::mutex pause_mutex_;

  /// @brief Condition variable that wakes #publish_thread_ when the music is
  /// resumed.
  std::condition_variable pause_cv_;

  /// @brief Publisher that sends audio chunks to the audio player.
  rclcpp::Publisher<audio_common_msgs::msg::AudioStamped>::SharedPtr
      player_pub_;

  /// @brief Service server that starts music playback.
  rclcpp::Service<audio_common_msgs::srv::MusicPlay>::SharedPtr play_service_;

  /// @brief Service server that stops music playback.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;

  /// @brief Service server that pauses music playback.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;

  /// @brief Service server that resumes paused music playback.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;

  /**
   * @brief Open @p file_path as a WAV file and publish its contents in chunks
   *        until the track ends, #stop_music_ is set, or the file cannot be
   *        read.  Respects #pause_music_ and #audio_loop_.
   *
   * Intended to run in #publish_thread_.
   *
   * @param file_path Absolute path to the WAV file to stream.
   */
  void publish_audio(const std::string &file_path);

  /**
   * @brief Service callback that begins playback of a requested audio file.
   *
   * If a track is already playing the request is rejected.  The file path may
   * be supplied directly via @c request->file_path, or resolved from the
   * package share directory using @c request->audio as a stem name.
   *
   * @param request  Service request containing the file path or audio name,
   *                 loop flag, and optional volume.
   * @param response Service response indicating success or failure.
   */
  void play_callback(
      const std::shared_ptr<audio_common_msgs::srv::MusicPlay::Request> request,
      std::shared_ptr<audio_common_msgs::srv::MusicPlay::Response> response);

  /**
   * @brief Service callback that pauses ongoing playback.
   *
   * Sets #pause_music_ so the publishing thread blocks at the next chunk
   * boundary.
   *
   * @param request  Unused trigger request.
   * @param response Trigger response (always success).
   */
  void
  pause_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Service callback that resumes paused playback.
   *
   * Clears #pause_music_ and notifies #pause_cv_ to unblock the publishing
   * thread.
   *
   * @param request  Unused trigger request.
   * @param response Trigger response (always success).
   */
  void resume_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Service callback that stops ongoing playback.
   *
   * Sets #stop_music_ so the publishing thread exits its loop after the
   * current chunk.
   *
   * @param request  Unused trigger request.
   * @param response Trigger response (always success).
   */
  void
  stop_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);
};

} // namespace audio_common

#endif