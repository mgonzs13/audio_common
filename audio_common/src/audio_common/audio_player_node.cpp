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
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <portaudio.h>
#include <rclcpp/rclcpp.hpp>

#include "audio_common/audio_player_node.hpp"
#include "audio_common_msgs/msg/audio.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"

using namespace audio_common;
using std::placeholders::_1;

namespace {

template <typename T> float sample_to_float(T sample) {
  return static_cast<float>(sample) /
         (static_cast<float>(std::numeric_limits<T>::max()) + 1.0f);
}

template <> float sample_to_float<float>(float sample) { return sample; }

template <> float sample_to_float<uint8_t>(uint8_t sample) {
  return (static_cast<float>(sample) - 128.0f) / 128.0f;
}

} // namespace

AudioPlayerNode::AudioPlayerNode() : Node("audio_player_node") {
  // Declare parameters
  this->declare_parameter<int>("channels", 2);
  this->declare_parameter<int>("rate", 0);
  this->declare_parameter<int>("device", -1);

  // Get parameters
  this->channels_ = this->get_parameter("channels").as_int();
  this->rate_ = this->get_parameter("rate").as_int();
  this->device_ = this->get_parameter("device").as_int();

  if (this->channels_ <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid output channel count: %d",
                 this->channels_);
    throw std::runtime_error("Invalid output channel count");
  }

  // Initialize PortAudio
  PaError err = Pa_Initialize();
  if (err != paNoError) {
    RCLCPP_ERROR(this->get_logger(), "PortAudio error: %s",
                 Pa_GetErrorText(err));
    throw std::runtime_error("Failed to initialize PortAudio");
  }

  // Subscription to audio topic
  auto qos_profile = rclcpp::SensorDataQoS();
  this->audio_sub_ =
      this->create_subscription<audio_common_msgs::msg::AudioStamped>(
          "audio", qos_profile,
          std::bind(&AudioPlayerNode::audio_callback, this, _1));

  RCLCPP_INFO(this->get_logger(), "AudioPlayer node started");
}

AudioPlayerNode::~AudioPlayerNode() {
  // Close all open streams and terminate PortAudio
  for (auto &stream_pair : this->stream_dict_) {
    Pa_StopStream(stream_pair.second.stream);
    Pa_CloseStream(stream_pair.second.stream);
  }
  Pa_Terminate();
}

void AudioPlayerNode::audio_callback(
    const audio_common_msgs::msg::AudioStamped::SharedPtr msg) {

  PaDeviceIndex output_device =
      (this->device_ >= 0) ? this->device_ : Pa_GetDefaultOutputDevice();

  if (output_device == paNoDevice) {
    RCLCPP_ERROR(this->get_logger(), "No PortAudio output device available");
    return;
  }

  const PaDeviceInfo *device_info = Pa_GetDeviceInfo(output_device);
  if (device_info == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Invalid PortAudio output device: %d",
                 output_device);
    return;
  }

  int output_rate = this->rate_ > 0
                        ? this->rate_
                        : static_cast<int>(device_info->defaultSampleRate);

  if (msg->audio.info.rate <= 0 || output_rate <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid sample rate: input=%d, output=%d",
                 msg->audio.info.rate, output_rate);
    return;
  }

  // Create a unique stream key based on input and output audio formats
  std::string stream_key = std::to_string(msg->audio.info.format) + "_" +
                           std::to_string(msg->audio.info.rate) + "_" +
                           std::to_string(msg->audio.info.channels) + "_" +
                           std::to_string(output_rate) + "_" +
                           std::to_string(this->channels_);

  // Check if stream already exists, if not, create one
  if (this->stream_dict_.find(stream_key) == this->stream_dict_.end()) {
    PaStreamParameters outputParameters;
    outputParameters.device = output_device;
    outputParameters.channelCount = this->channels_;
    outputParameters.sampleFormat = paFloat32;
    outputParameters.suggestedLatency = device_info->defaultHighOutputLatency;
    outputParameters.hostApiSpecificStreamInfo = nullptr;

    PaError err = Pa_IsFormatSupported(nullptr, &outputParameters, output_rate);

    if (err != paFormatIsSupported) {
      RCLCPP_ERROR(this->get_logger(),
                   "Output device %d does not support %d Hz: %s", output_device,
                   output_rate, Pa_GetErrorText(err));
      return;
    }

    PlaybackStream playback_stream{};
    playback_stream.stream = nullptr;
    playback_stream.input_rate = msg->audio.info.rate;
    playback_stream.output_rate = output_rate;

    err = Pa_OpenStream(&playback_stream.stream, nullptr, &outputParameters,
                        output_rate, 1024, paClipOff, nullptr, nullptr);

    if (err != paNoError) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open audio stream: %s",
                   Pa_GetErrorText(err));
      return;
    }

    err = Pa_StartStream(playback_stream.stream);
    if (err != paNoError) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start audio stream: %s",
                   Pa_GetErrorText(err));
      Pa_CloseStream(playback_stream.stream);
      return;
    }

    this->stream_dict_[stream_key] = playback_stream;
  }

  // Write audio from ROS 2 msg
  switch (msg->audio.info.format) {
  case paFloat32:
    this->write_data(msg->audio.audio_data.float32_data,
                     msg->audio.info.channels, msg->audio.info.rate,
                     msg->audio.info.chunk, stream_key);
    break;

  case paInt32:
    this->write_data(msg->audio.audio_data.int32_data, msg->audio.info.channels,
                     msg->audio.info.rate, msg->audio.info.chunk, stream_key);
    break;

  case paInt16:
    this->write_data(msg->audio.audio_data.int16_data, msg->audio.info.channels,
                     msg->audio.info.rate, msg->audio.info.chunk, stream_key);
    break;

  case paInt8:
    this->write_data(msg->audio.audio_data.int8_data, msg->audio.info.channels,
                     msg->audio.info.rate, msg->audio.info.chunk, stream_key);
    break;

  case paUInt8:
    this->write_data(msg->audio.audio_data.uint8_data, msg->audio.info.channels,
                     msg->audio.info.rate, msg->audio.info.chunk, stream_key);
    break;
  default:
    RCLCPP_ERROR(this->get_logger(), "Unsupported format");
    return;
  }
}

template <typename ContainerT>
void AudioPlayerNode::write_data(const ContainerT &input_data, int channels,
                                 int rate, int chunk,
                                 const std::string &stream_key) {

  auto stream_it = this->stream_dict_.find(stream_key);
  if (stream_it == this->stream_dict_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Audio stream not found");
    return;
  }

  if (channels <= 0 || chunk <= 0) {
    RCLCPP_WARN(this->get_logger(), "Invalid audio data shape");
    return;
  }

  const size_t input_frames = static_cast<size_t>(chunk);
  const size_t required_input_samples = input_frames * channels;

  if (input_data.size() < required_input_samples) {
    RCLCPP_WARN(this->get_logger(),
                "Insufficient data (%zu) for requested chunk size (%zu).",
                input_data.size(), required_input_samples);
    return;
  }

  std::vector<float> data(input_frames * this->channels_);

  // Handle mono-to-stereo or stereo-to-mono conversions if necessary
  if (channels != this->channels_) {
    if (channels == 1 && this->channels_ == 2) {
      // Mono to stereo conversion
      for (size_t i = 0; i < input_frames; ++i) {
        const float sample = sample_to_float(input_data[i]);
        data[2 * i] = sample;
        data[2 * i + 1] = sample;
      }
    } else if (channels == 2 && this->channels_ == 1) {
      // Stereo to mono conversion
      for (size_t i = 0; i < input_frames; ++i) {
        data[i] = (sample_to_float(input_data[2 * i]) +
                   sample_to_float(input_data[2 * i + 1])) /
                  2.0f;
      }
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Unsupported channel conversion from %d to %d channels.",
                  channels, this->channels_);
      return;
    }
  } else {
    // No conversion needed
    for (size_t i = 0; i < data.size(); ++i) {
      data[i] = sample_to_float(input_data[i]);
    }
  }

  const float *write_data = data.data();
  size_t total_frames = input_frames;
  std::vector<float> resampled_data;

  if (rate != stream_it->second.output_rate) {
    resampled_data = stream_it->second.sample_rate_converter.convert(
        data, this->channels_, rate, stream_it->second.output_rate);
    write_data = resampled_data.data();
    total_frames = resampled_data.size() / this->channels_;
  }

  if (total_frames == 0) {
    return;
  }

  // Write in smaller blocks to reduce underrun risk
  size_t frames_written = 0;
  const size_t max_block = 1024;

  while (frames_written < total_frames) {
    size_t frames_to_write = std::min(max_block, total_frames - frames_written);
    PaError err = Pa_WriteStream(stream_it->second.stream,
                                 write_data + frames_written * this->channels_,
                                 frames_to_write);

    if (err == paOutputUnderflowed) {
      RCLCPP_WARN(this->get_logger(),
                  "PortAudio underrun detected, retrying...");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue; // Try again this block

    } else if (err != paNoError) {
      RCLCPP_ERROR(this->get_logger(), "PortAudio write error: %s",
                   Pa_GetErrorText(err));
      break;
    }

    frames_written += frames_to_write;
  }
}
