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

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include "audio_aec/audio_converter.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"

using std::placeholders::_1;

namespace audio_aec {
namespace {

constexpr int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr int64_t kThrottleMs = 1000;

bool is_supported_rate(int rate) {
  return rate == 8000 || rate == 16000 || rate == 32000 || rate == 48000;
}

bool is_supported_format(int format) {
  return format == kPaFloat32 || format == kPaInt32 || format == kPaInt16 ||
         format == kPaInt8 || format == kPaUInt8;
}

int64_t stamp_to_ns(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<int64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<int64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time ns_to_time(int64_t ns) {
  if (ns < 0) {
    ns = 0;
  }

  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(ns / kNanosecondsPerSecond);
  stamp.nanosec = static_cast<uint32_t>(ns % kNanosecondsPerSecond);
  return stamp;
}

} // namespace

AecNode::AecNode(const rclcpp::NodeOptions &options)
    : Node("aec_node", options), format_(kPaInt16),
      last_reference_reset_count_(0) {
  // Declare parameters with default values
  this->declare_parameter<std::string>("mic_topic", "audio");
  this->declare_parameter<std::string>("reference_topic", "audio_reference");
  this->declare_parameter<std::string>("output_topic", "audio_clean");
  this->declare_parameter<int>("rate", 16000);
  this->declare_parameter<int>("format", kPaInt16);
  this->declare_parameter<int>("mic_buffer_ms", 100);
  this->declare_parameter<int>("output_chunk_ms", 20);
  this->declare_parameter<int>("stream_delay_ms", 0);
  this->declare_parameter<bool>("enable_high_pass", true);
  this->declare_parameter<bool>("enable_noise_suppression", false);
  this->declare_parameter<bool>("enable_agc", false);
  this->declare_parameter<int>("reference_timeout_ms", 500);

  // Get parameters
  const std::string mic_topic = this->get_parameter("mic_topic").as_string();
  const std::string reference_topic =
      this->get_parameter("reference_topic").as_string();
  const std::string output_topic =
      this->get_parameter("output_topic").as_string();
  const int rate = this->get_parameter("rate").as_int();
  this->format_ = this->get_parameter("format").as_int();
  const int mic_buffer_ms = this->get_parameter("mic_buffer_ms").as_int();
  const int output_chunk_ms = this->get_parameter("output_chunk_ms").as_int();
  const int stream_delay_ms = this->get_parameter("stream_delay_ms").as_int();
  const bool enable_high_pass =
      this->get_parameter("enable_high_pass").as_bool();
  const bool enable_noise_suppression =
      this->get_parameter("enable_noise_suppression").as_bool();
  const bool enable_agc = this->get_parameter("enable_agc").as_bool();
  const int reference_timeout_ms =
      this->get_parameter("reference_timeout_ms").as_int();

  // Validate parameters
  if (!is_supported_rate(rate)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid parameter 'rate': %d (must be 8000, 16000, 32000, "
                 "or 48000)",
                 rate);
    throw std::runtime_error("Invalid parameter 'rate'");
  }

  if (!is_supported_format(this->format_)) {
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
          mic_topic, rclcpp::SensorDataQoS(),
          std::bind(&AecNode::mic_callback, this, _1));

  this->reference_sub_ =
      this->create_subscription<audio_common_msgs::msg::AudioStamped>(
          reference_topic, rclcpp::SensorDataQoS(),
          std::bind(&AecNode::reference_callback, this, _1));

  this->output_pub_ =
      this->create_publisher<audio_common_msgs::msg::AudioStamped>(
          output_topic, rclcpp::SensorDataQoS());

  RCLCPP_INFO(this->get_logger(), "AEC node started (rate=%d Hz, format=%d)",
              rate, this->format_);
}

void AecNode::mic_callback(
    const audio_common_msgs::msg::AudioStamped::SharedPtr msg) {
  const audio_common_msgs::msg::AudioInfo &info = msg->audio.info;

  if (info.channels <= 0 || info.rate <= 0 ||
      !is_supported_format(info.format)) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), kThrottleMs,
        "Dropping microphone message with invalid info (channels=%d, rate=%d, "
        "format=%d)",
        info.channels, info.rate, info.format);
    return;
  }

  std::vector<float> mono;
  try {
    mono = AudioConverter::to_mono_float(msg->audio.audio_data, info.format,
                                         info.channels);
  } catch (const std::invalid_argument &e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), kThrottleMs,
                          "Dropping microphone message: %s", e.what());
    return;
  }

  int64_t stamp_ns = stamp_to_ns(msg->header.stamp);
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
      !is_supported_format(info.format)) {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), kThrottleMs,
        "Dropping reference message with invalid info (channels=%d, rate=%d, "
        "format=%d)",
        info.channels, info.rate, info.format);
    return;
  }

  std::vector<float> mono;
  try {
    mono = AudioConverter::to_mono_float(msg->audio.audio_data, info.format,
                                         info.channels);
  } catch (const std::invalid_argument &e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), kThrottleMs,
                          "Dropping reference message: %s", e.what());
    return;
  }

  int64_t stamp_ns = stamp_to_ns(msg->header.stamp);
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
    msg.header.stamp = ns_to_time(chunk.stamp_ns);
    msg.audio.info.format = this->format_;
    msg.audio.info.channels = 1;
    msg.audio.info.rate = this->processor_->rate();
    msg.audio.info.chunk = static_cast<int32_t>(chunk.samples.size());

    AudioConverter::from_mono_float(chunk.samples, this->format_,
                                    msg.audio.audio_data);
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
