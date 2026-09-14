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

#include <cstdint>
#include <memory>
#include <portaudio.h>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

#include "audio_common/audio_capturer_node.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"

using namespace audio_common;

AudioCapturerNode::AudioCapturerNode() : Node("audio_capturer_node") {

  // Declare parameters with default values
  this->declare_parameter<int>("format", paInt16);
  this->declare_parameter<int>("channels", 1);
  this->declare_parameter<int>("rate", 16000);
  this->declare_parameter<int>("chunk", 512);
  this->declare_parameter<int>("device", -1);
  this->declare_parameter<std::string>("frame_id", "");

  // Get parameters
  this->format_ = this->get_parameter("format").as_int();
  this->channels_ = this->get_parameter("channels").as_int();
  this->rate_ = this->get_parameter("rate").as_int();
  this->chunk_ = this->get_parameter("chunk").as_int();
  int device = this->get_parameter("device").as_int();
  this->frame_id_ = this->get_parameter("frame_id").as_string();

  // Initialize PortAudio
  PaError err = Pa_Initialize();
  if (err != paNoError) {
    RCLCPP_ERROR(this->get_logger(), "PortAudio error: %s",
                 Pa_GetErrorText(err));
    throw std::runtime_error("Failed to initialize PortAudio");
  }

  if (this->channels_ <= 0 || this->rate_ <= 0 || this->chunk_ <= 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid capture parameters: channels=%d, rate=%d, chunk=%d",
                 this->channels_, this->rate_, this->chunk_);
    throw std::runtime_error("Invalid capture parameters");
  }

  PaStreamParameters inputParameters;
  inputParameters.device = (device >= 0) ? device : Pa_GetDefaultInputDevice();

  if (inputParameters.device == paNoDevice) {
    RCLCPP_ERROR(this->get_logger(), "No PortAudio input device available");
    throw std::runtime_error("No PortAudio input device available");
  }

  const PaDeviceInfo *device_info = Pa_GetDeviceInfo(inputParameters.device);
  if (device_info == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Invalid PortAudio input device: %d",
                 inputParameters.device);
    throw std::runtime_error("Invalid PortAudio input device");
  }

  inputParameters.channelCount = this->channels_;
  inputParameters.sampleFormat = this->format_;
  inputParameters.suggestedLatency = device_info->defaultLowInputLatency;
  inputParameters.hostApiSpecificStreamInfo = nullptr;

  err = Pa_OpenStream(&this->stream_, &inputParameters,
                      nullptr, // output parameters (not used)
                      this->rate_, this->chunk_, paClipOff, nullptr, nullptr);

  if (err != paNoError) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open audio stream: %s",
                 Pa_GetErrorText(err));
    throw std::runtime_error("Failed to open PortAudio stream");
  }

  err = Pa_StartStream(this->stream_);
  if (err != paNoError) {
    RCLCPP_ERROR(this->get_logger(), "Failed to start audio stream: %s",
                 Pa_GetErrorText(err));
    Pa_CloseStream(this->stream_);
    this->stream_ = nullptr;
    throw std::runtime_error("Failed to start PortAudio stream");
  }

  this->audio_pub_ =
      this->create_publisher<audio_common_msgs::msg::AudioStamped>(
          "audio", rclcpp::SensorDataQoS());

  RCLCPP_INFO(this->get_logger(), "AudioCapturer node started");
}

AudioCapturerNode::~AudioCapturerNode() {
  if (this->stream_ != nullptr) {
    Pa_StopStream(this->stream_);
    Pa_CloseStream(this->stream_);
  }
  Pa_Terminate();
}

void AudioCapturerNode::work() {
  while (rclcpp::ok()) {

    auto msg = audio_common_msgs::msg::AudioStamped();
    msg.header.frame_id = this->frame_id_;
    msg.header.stamp = this->get_clock()->now();

    bool read_ok = false;
    switch (this->format_) {
    case paFloat32:
      read_ok = this->read_data<float>(msg.audio.audio_data.float32_data);
      break;
    case paInt32:
      read_ok = this->read_data<int32_t>(msg.audio.audio_data.int32_data);
      break;
    case paInt16:
      read_ok = this->read_data<int16_t>(msg.audio.audio_data.int16_data);
      break;
    case paInt8:
      read_ok = this->read_data<int8_t>(msg.audio.audio_data.int8_data);
      break;
    case paUInt8:
      read_ok = this->read_data<uint8_t>(msg.audio.audio_data.uint8_data);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unsupported format");
      continue;
    }

    if (!read_ok) {
      continue;
    }

    msg.audio.info.format = this->format_;
    msg.audio.info.channels = this->channels_;
    msg.audio.info.chunk = this->chunk_;
    msg.audio.info.rate = this->rate_;

    this->audio_pub_->publish(msg);
  }
}

template <typename T> bool AudioCapturerNode::read_data(std::vector<T> &data) {
  data.resize(this->chunk_ * this->channels_);
  PaError err = Pa_ReadStream(this->stream_, data.data(), this->chunk_);

  if (err == paInputOverflowed) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "PortAudio input overflow detected");
  } else if (err != paNoError) {
    RCLCPP_ERROR(this->get_logger(), "PortAudio read error: %s",
                 Pa_GetErrorText(err));
    data.clear();
    return false;
  }

  return true;
}
