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

#include "audio_aec/aec_node.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "audio_common_dsp/audio_converter.hpp"
#include "audio_common_dsp/time_utils.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"

using std::placeholders::_1;

namespace audio_aec {
namespace {

constexpr int64_t kThrottleMs = 1000;

} // namespace

AecNode::AecNode(const rclcpp::NodeOptions &options)
    : Node("aec_node", options), format_(audio_common_dsp::kPaInt16),
      last_reference_reset_count_(0) {
  // Declare parameters with default values
  this->declare_parameter<int>("rate", 16000);
  this->declare_parameter<int>("format", audio_common_dsp::kPaInt16);
  this->declare_parameter<int>("mic_buffer_ms", 100);
  this->declare_parameter<int>("output_chunk_ms", 20);
  this->declare_parameter<int>("stream_delay_ms", 0);
  this->declare_parameter<int>("reference_lead_ms", 0);
  this->declare_parameter<int>("reference_wait_ms", 100);
  this->declare_parameter<bool>("enable_high_pass", true);
  this->declare_parameter<bool>("enable_noise_suppression", false);
  this->declare_parameter<bool>("enable_agc", false);
  this->declare_parameter<int>("reference_timeout_ms", 500);

  // Get parameters
  const int rate = this->get_parameter("rate").as_int();
  this->format_ = this->get_parameter("format").as_int();
  const int mic_buffer_ms = this->get_parameter("mic_buffer_ms").as_int();
  const int output_chunk_ms = this->get_parameter("output_chunk_ms").as_int();
  const int stream_delay_ms = this->get_parameter("stream_delay_ms").as_int();
  const int reference_lead_ms =
      this->get_parameter("reference_lead_ms").as_int();
  const int reference_wait_ms =
      this->get_parameter("reference_wait_ms").as_int();
  const bool enable_high_pass =
      this->get_parameter("enable_high_pass").as_bool();
  const bool enable_noise_suppression =
      this->get_parameter("enable_noise_suppression").as_bool();
  const bool enable_agc = this->get_parameter("enable_agc").as_bool();
  const int reference_timeout_ms =
      this->get_parameter("reference_timeout_ms").as_int();

  // Validate parameters
  if (!AecProcessor::is_supported_rate(rate)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid parameter 'rate': %d (must be 8000, 16000, 32000, "
                 "or 48000)",
                 rate);
    throw std::runtime_error("Invalid parameter 'rate'");
  }

  if (!audio_common_dsp::AudioConverter::is_supported_format(this->format_)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid parameter 'format': %d (must be 1, 2, 8, 16, or 32)",
                 this->format_);
    throw std::runtime_error("Invalid parameter 'format'");
  }

  if (mic_buffer_ms < 0 || mic_buffer_ms % 10 != 0 || mic_buffer_ms > 500) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Invalid parameter 'mic_buffer_ms': %d (must be non-negative, a "
        "multiple of 10, and at most 500)",
        mic_buffer_ms);
    throw std::runtime_error("Invalid parameter 'mic_buffer_ms'");
  }

  if (output_chunk_ms <= 0 || output_chunk_ms % 10 != 0) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Invalid parameter 'output_chunk_ms': %d (must be positive and a "
        "multiple of 10)",
        output_chunk_ms);
    throw std::runtime_error("Invalid parameter 'output_chunk_ms'");
  }

  if (stream_delay_ms < 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid parameter 'stream_delay_ms': %d (must be "
                 "non-negative)",
                 stream_delay_ms);
    throw std::runtime_error("Invalid parameter 'stream_delay_ms'");
  }

  if (reference_lead_ms < 0 || reference_lead_ms > 500) {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid parameter 'reference_lead_ms': %d (must be between "
                 "0 and 500)",
                 reference_lead_ms);
    throw std::runtime_error("Invalid parameter 'reference_lead_ms'");
  }

  if (reference_wait_ms < 0 || reference_wait_ms > 1000) {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid parameter 'reference_wait_ms': %d (must be between "
                 "0 and 1000)",
                 reference_wait_ms);
    throw std::runtime_error("Invalid parameter 'reference_wait_ms'");
  }

  if (reference_timeout_ms <= 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid parameter 'reference_timeout_ms': %d (must be "
                 "positive)",
                 reference_timeout_ms);
    throw std::runtime_error("Invalid parameter 'reference_timeout_ms'");
  }

  // Create the processing pipeline
  AecProcessorConfig config;
  config.rate = rate;
  config.mic_buffer_ms = mic_buffer_ms;
  config.output_chunk_ms = output_chunk_ms;
  config.stream_delay_ms = stream_delay_ms;
  config.reference_lead_ms = reference_lead_ms;
  config.reference_wait_ms = reference_wait_ms;
  config.enable_high_pass = enable_high_pass;
  config.enable_noise_suppression = enable_noise_suppression;
  config.enable_agc = enable_agc;
  config.reference_timeout_ms = reference_timeout_ms;

  try {
    this->processor_ = std::unique_ptr<AecProcessor>(new AecProcessor(config));
  } catch (const std::invalid_argument &e) {
    RCLCPP_ERROR(this->get_logger(), "Invalid AEC configuration: %s", e.what());
    throw std::runtime_error(e.what());
  }

  // Create ROS interfaces
  this->mic_sub_ =
      this->create_subscription<audio_common_msgs::msg::AudioStamped>(
          "audio", rclcpp::SensorDataQoS(),
          std::bind(&AecNode::mic_callback, this, _1));

  this->reference_sub_ =
      this->create_subscription<audio_common_msgs::msg::AudioStamped>(
          "audio_reference", rclcpp::SensorDataQoS(),
          std::bind(&AecNode::reference_callback, this, _1));

  this->output_pub_ =
      this->create_publisher<audio_common_msgs::msg::AudioStamped>(
          "audio_clean", rclcpp::SensorDataQoS());

  RCLCPP_INFO(this->get_logger(), "AEC node started (rate=%d Hz, format=%d)",
              rate, this->format_);
}

void AecNode::mic_callback(
    const audio_common_msgs::msg::AudioStamped::SharedPtr msg) {
  const audio_common_msgs::msg::AudioInfo &info = msg->audio.info;

  if (info.channels <= 0 || info.rate <= 0 ||
      !audio_common_dsp::AudioConverter::is_supported_format(info.format)) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), kThrottleMs,
        "Dropping microphone message with invalid info (channels=%d, rate=%d, "
        "format=%d)",
        info.channels, info.rate, info.format);
    return;
  }

  std::vector<float> mono;
  try {
    mono = audio_common_dsp::AudioConverter::to_mono_float(
        msg->audio.audio_data, info.format, info.channels);
  } catch (const std::invalid_argument &e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), kThrottleMs,
                          "Dropping microphone message: %s", e.what());
    return;
  }

  int64_t stamp_ns = audio_common_dsp::to_nanoseconds(
      msg->header.stamp.sec, msg->header.stamp.nanosec);
  if (stamp_ns == 0) {
    stamp_ns = this->now().nanoseconds();
  }

  std::lock_guard<std::mutex> lock(this->mutex_);
  this->latest_frame_id_ = msg->header.frame_id;
  this->processor_->push_mic(mono, info.rate, stamp_ns);
  this->publish_ready();
}

void AecNode::reference_callback(
    const audio_common_msgs::msg::AudioStamped::SharedPtr msg) {
  const audio_common_msgs::msg::AudioInfo &info = msg->audio.info;

  if (info.channels <= 0 || info.rate <= 0 ||
      !audio_common_dsp::AudioConverter::is_supported_format(info.format)) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), kThrottleMs,
        "Dropping reference message with invalid info (channels=%d, rate=%d, "
        "format=%d)",
        info.channels, info.rate, info.format);
    return;
  }

  std::vector<float> mono;
  try {
    mono = audio_common_dsp::AudioConverter::to_mono_float(
        msg->audio.audio_data, info.format, info.channels);
  } catch (const std::invalid_argument &e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), kThrottleMs,
                          "Dropping reference message: %s", e.what());
    return;
  }

  int64_t stamp_ns = audio_common_dsp::to_nanoseconds(
      msg->header.stamp.sec, msg->header.stamp.nanosec);
  if (stamp_ns == 0) {
    stamp_ns = this->now().nanoseconds();
  }

  std::lock_guard<std::mutex> lock(this->mutex_);
  this->processor_->push_reference(mono, info.rate, stamp_ns);
  this->publish_ready();
}

void AecNode::publish_ready() {
  const std::vector<OutputChunk> chunks = this->processor_->pop_output();

  for (const OutputChunk &chunk : chunks) {
    audio_common_msgs::msg::AudioStamped msg;
    msg.header.frame_id = this->latest_frame_id_;
    msg.header.stamp.sec = 0;
    msg.header.stamp.nanosec = 0;
    audio_common_dsp::from_nanoseconds(chunk.stamp_ns, msg.header.stamp.sec,
                                       msg.header.stamp.nanosec);
    msg.audio.info.format = this->format_;
    msg.audio.info.channels = 1;
    msg.audio.info.rate = this->processor_->rate();
    msg.audio.info.chunk = static_cast<int32_t>(chunk.samples.size());

    audio_common_dsp::AudioConverter::from_mono_float(
        chunk.samples, this->format_, msg.audio.audio_data);
    this->output_pub_->publish(msg);
  }

  const uint64_t reference_reset_count =
      this->processor_->reference_reset_count();
  if (reference_reset_count != this->last_reference_reset_count_) {
    this->last_reference_reset_count_ = reference_reset_count;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), kThrottleMs,
                         "Reference audio timed out; resetting the AEC");
  }

  if (this->processor_->had_error()) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), kThrottleMs,
        "AEC processing error (WebRTC code %d); some audio was dropped or "
        "processed without echo cancellation",
        this->processor_->last_error());
    this->processor_->clear_error();
  }
}

} // namespace audio_aec
