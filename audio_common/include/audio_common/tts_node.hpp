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

#ifndef AUDIO_COMMON__TTS_NODE
#define AUDIO_COMMON__TTS_NODE

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include <chrono>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "audio_common/wave_file.hpp"
#include "audio_common_msgs/action/tts.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"

namespace audio_common {

/**
 * @brief ROS 2 node that converts text to speech using @c espeak and streams
 *        the resulting audio via a ROS 2 action server.
 *
 * When a TTS action goal is received the node invokes @c espeak to produce a
 * temporary WAV file, then reads and publishes the file as @c AudioStamped
 * messages at the file's natural playback rate.  Per-chunk feedback is
 * published so clients can track progress.  Any active goal is aborted when a
 * new goal is accepted.
 *
 * @par ROS 2 Parameters
 * - `chunk`    (int, default 4096): Number of float samples per published
 *              message.
 * - `frame_id` (string, default ""): TF frame ID attached to published
 *              message headers.
 *
 * @par Publications
 * - `audio` (audio_common_msgs/msg/AudioStamped, SensorDataQoS)
 *
 * @par Action Server
 * - `say` (audio_common_msgs/action/TTS)
 */
class TtsNode : public rclcpp::Node {
public:
  /// Convenience alias for the TTS action type.
  using TTS = audio_common_msgs::action::TTS;

  /// Convenience alias for the goal-handle type used by the action server.
  using GoalHandleTTS = rclcpp_action::ServerGoalHandle<TTS>;

  /**
   * @brief Construct the node, declare/read parameters, create the publisher
   *        and the action server.
   */
  TtsNode();

private:
  /// @brief Number of float samples per published audio message.
  /// Corresponds to the ROS 2 parameter \"chunk\".
  int chunk_;

  /// @brief TF frame ID used in published message headers.
  /// Corresponds to the ROS 2 parameter \"frame_id\".
  std::string frame_id_;

  /// @brief Publisher that sends synthesised audio chunks to the audio player.
  rclcpp::Publisher<audio_common_msgs::msg::AudioStamped>::SharedPtr
      player_pub_;

  /// @brief Action server that handles incoming TTS goals.
  rclcpp_action::Server<TTS>::SharedPtr action_server_;

  /// @brief Mutex that protects #goal_handle_ during goal handover.
  std::mutex goal_lock_;

  /// @brief Handle to the currently active (or last accepted) TTS goal.
  std::shared_ptr<GoalHandleTTS> goal_handle_;

  /**
   * @brief Action server callback – decides whether to accept an incoming
   *        goal.
   *
   * All goals are accepted unconditionally; any currently active goal is
   * aborted inside handle_accepted().
   *
   * @param uuid Unique identifier of the incoming goal (unused).
   * @param goal Shared pointer to the goal message.
   * @return `ACCEPT_AND_EXECUTE` always.
   */
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const TTS::Goal> goal);

  /**
   * @brief Action server callback – decides whether to accept a cancel
   *        request for an active goal.
   *
   * @param goal_handle Handle to the goal that is requested to be cancelled.
   * @return `ACCEPT` always.
   */
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleTTS> goal_handle);

  /**
   * @brief Action server callback – called when a new goal has been accepted.
   *
   * Aborts any existing active goal and spawns a detached thread to execute
   * the new goal via execute_callback().
   *
   * @param goal_handle Handle to the newly accepted goal.
   */
  void handle_accepted(const std::shared_ptr<GoalHandleTTS> goal_handle);

  /**
   * @brief Executes a TTS goal in a background thread.
   *
   * Synthesises speech with @c espeak, reads the resulting WAV file in chunks,
   * publishes each chunk as an @c AudioStamped message, and sends per-chunk
   * feedback.  Sets the goal result to the synthesised text on success.
   *
   * @param goal_handle Handle to the goal being executed.
   */
  void execute_callback(const std::shared_ptr<GoalHandleTTS> goal_handle);
};

} // namespace audio_common

#endif