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

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "audio_common/tts_node.hpp"
#include "audio_common/wave_file.hpp"
#include "audio_common_msgs/action/tts.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"

using namespace audio_common;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace {

bool is_valid_language(const std::string &language) {
  if (language.empty()) {
    return false;
  }
  return std::all_of(language.begin(), language.end(), [](unsigned char c) {
    return std::isalnum(c) != 0 || c == '-' || c == '_' || c == '+';
  });
}

/// @brief Removes a file on scope exit.
class TempFile {
public:
  ~TempFile() {
    if (!this->path.empty()) {
      std::remove(this->path.c_str());
    }
  }

  std::string path;
};

} // namespace

TtsNode::TtsNode() : Node("tts_node") {

  this->declare_parameter("chunk", 4096);
  this->declare_parameter("frame_id", "");

  this->chunk_ = this->get_parameter("chunk").as_int();
  this->frame_id_ = this->get_parameter("frame_id").as_string();

  if (this->chunk_ <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid chunk size: %d", this->chunk_);
    throw std::runtime_error("Invalid chunk size");
  }

  this->player_pub_ =
      this->create_publisher<audio_common_msgs::msg::AudioStamped>(
          "audio", rclcpp::SensorDataQoS());

  // Action server
  this->action_server_ = rclcpp_action::create_server<TTS>(
      this, "say", std::bind(&TtsNode::handle_goal, this, _1, _2),
      std::bind(&TtsNode::handle_cancel, this, _1),
      std::bind(&TtsNode::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "TTS node started");
}

TtsNode::~TtsNode() {
  std::unique_lock<std::mutex> lock(this->goal_lock_);

  if (this->goal_handle_ != nullptr && this->goal_handle_->is_active()) {
    auto result = std::make_shared<TTS::Result>();
    this->goal_handle_->abort(result);
  }

  if (this->worker_.joinable()) {
    this->worker_.join();
  }
}

rclcpp_action::GoalResponse
TtsNode::handle_goal(const rclcpp_action::GoalUUID &uuid,
                     std::shared_ptr<const TTS::Goal> goal) {
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
TtsNode::handle_cancel(const std::shared_ptr<GoalHandleTTS> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Canceling TTS...");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TtsNode::handle_accepted(
    const std::shared_ptr<GoalHandleTTS> goal_handle) {
  std::unique_lock<std::mutex> lock(this->goal_lock_);
  if (this->goal_handle_ != nullptr && this->goal_handle_->is_active()) {
    auto result = std::make_shared<TTS::Result>();
    this->goal_handle_->abort(result);
  }

  if (this->worker_.joinable()) {
    this->worker_.join();
  }

  this->goal_handle_ = goal_handle;
  this->worker_ = std::thread(&TtsNode::execute_callback, this, goal_handle);
}

void TtsNode::execute_callback(
    const std::shared_ptr<GoalHandleTTS> goal_handle) {
  auto result = std::make_shared<TTS::Result>();
  const auto goal = goal_handle->get_goal();
  const std::string text = goal->text;
  const std::string language = goal->language;
  const int rate =
      std::min(450, std::max(80, static_cast<int>(goal->rate * 175)));
  const int volume =
      std::min(200, std::max(0, static_cast<int>(goal->volume * 100)));

  if (!is_valid_language(language)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid TTS language: '%s'",
                 language.c_str());
    goal_handle->abort(result);
    return;
  }

  // Write the text to a unique temporary file so it is never interpreted by
  // the shell and concurrent goals cannot clobber each other.
  TempFile text_file;
  {
    char path[] = "/tmp/tts_text_XXXXXX";
    const int fd = ::mkstemp(path);
    if (fd < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create temporary text file");
      goal_handle->abort(result);
      return;
    }
    text_file.path = path;

    const ssize_t written = ::write(fd, text.data(), text.size());
    ::close(fd);
    if (written < 0 || static_cast<size_t>(written) != text.size()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write temporary text file");
      goal_handle->abort(result);
      return;
    }
  }

  TempFile wav_file;
  {
    char path[] = "/tmp/tts_audio_XXXXXX";
    const int fd = ::mkstemp(path);
    if (fd < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create temporary audio file");
      goal_handle->abort(result);
      return;
    }
    wav_file.path = path;
    ::close(fd);
  }

  // Create audio file using espeak
  std::string cmd = "espeak -v" + language + " -s" + std::to_string(rate) +
                    " -a" + std::to_string(volume) + " -f " + text_file.path +
                    " -w " + wav_file.path;

  int ret = std::system(cmd.c_str());
  if (ret != 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "espeak command failed with return code: %d", ret);
    goal_handle->abort(result);
    return;
  }

  // Read audio file
  audio_common::WaveFile wf(wav_file.path);
  if (!wf.open()) {
    RCLCPP_ERROR(this->get_logger(), "Error opening audio file: %s",
                 wav_file.path.c_str());
    goal_handle->abort(result);
    return;
  }

  // Create rate
  std::chrono::nanoseconds period(
      (int)(1e9 * this->chunk_ / wf.get_sample_rate()));
  rclcpp::Rate pub_rate(period);
  std::vector<float> data(this->chunk_);

  // Publish the audio data in chunks
  while (wf.read(data, this->chunk_)) {
    if (!goal_handle->is_active()) {
      return;
    }

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }

    auto msg = audio_common_msgs::msg::AudioStamped();
    msg.header.frame_id = this->frame_id_;
    msg.header.stamp = this->get_clock()->now();
    msg.audio.audio_data.float32_data = data;
    msg.audio.info.channels = wf.get_num_channels();
    msg.audio.info.chunk =
        static_cast<int>(data.size() / wf.get_num_channels());
    msg.audio.info.format = 1;
    msg.audio.info.rate = wf.get_sample_rate();

    auto feedback = std::make_shared<TTS::Feedback>();
    feedback->audio = msg;

    this->player_pub_->publish(msg);
    goal_handle->publish_feedback(feedback);
    pub_rate.sleep();
  }

  result->text = text;
  goal_handle->succeed(result);
}
