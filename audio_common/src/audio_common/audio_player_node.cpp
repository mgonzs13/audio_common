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
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <portaudio.h>
#include <rclcpp/rclcpp.hpp>

#include "audio_common/audio_player_node.hpp"
#include "audio_common/audio_player_reference.hpp"
#include "audio_common_dsp/audio_converter.hpp"
#include "audio_common_dsp/time_utils.hpp"
#include "audio_common_msgs/msg/audio.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"

using namespace audio_common;
using std::placeholders::_1;

AudioPlayerNode::AudioPlayerNode() : Node("audio_player_node") {
  // Declare parameters
  this->declare_parameter<int>("channels", 2);
  this->declare_parameter<int>("rate", 0);
  this->declare_parameter<int>("device", -1);
  this->declare_parameter<std::string>("reference_topic", "played");
  this->declare_parameter<bool>("publish_reference", true);

  // Get parameters
  this->channels_ = this->get_parameter("channels").as_int();
  this->rate_ = this->get_parameter("rate").as_int();
  this->device_ = this->get_parameter("device").as_int();
  const std::string reference_topic =
      this->get_parameter("reference_topic").as_string();
  const bool publish_reference =
      this->get_parameter("publish_reference").as_bool();

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

  // Publication of the exact frames written to the output device.
  if (publish_reference && !reference_topic.empty()) {
    this->reference_pub_ =
        this->create_publisher<audio_common_msgs::msg::AudioStamped>(
            reference_topic, qos_profile);
  }

  RCLCPP_INFO(this->get_logger(), "AudioPlayer node started");
}

AudioPlayerNode::~AudioPlayerNode() {
  // Stop PortAudio first, then drain and stop the reference publisher.
  for (auto &stream_pair : this->stream_dict_) {
    auto &stream = *stream_pair.second;
    if (stream.stream != nullptr) {
      Pa_StopStream(stream.stream);
    }
    stream.stop.store(true);
    stream.reference_cv.notify_all();
    if (stream.reference_worker.joinable()) {
      stream.reference_worker.join();
    }
    if (stream.stream != nullptr) {
      Pa_CloseStream(stream.stream);
    }
  }
  this->stream_dict_.clear();
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

    auto playback_stream = std::make_unique<PlaybackStream>();
    playback_stream->output_rate = output_rate;
    playback_stream->channels = this->channels_;
    playback_stream->owner = this;

    err = Pa_OpenStream(
        &playback_stream->stream, nullptr, &outputParameters, output_rate, 4096,
        paClipOff, &AudioPlayerNode::playback_callback, playback_stream.get());

    if (err != paNoError) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open audio stream: %s",
                   Pa_GetErrorText(err));
      return;
    }

    err = Pa_StartStream(playback_stream->stream);
    if (err != paNoError) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start audio stream: %s",
                   Pa_GetErrorText(err));
      Pa_CloseStream(playback_stream->stream);
      return;
    }

    playback_stream->reference_worker = std::thread(
        &AudioPlayerNode::reference_loop, this, playback_stream.get());
    this->stream_dict_[stream_key] = std::move(playback_stream);
  }

  // Write audio from ROS 2 msg
  this->write_data(msg->audio.audio_data, msg->audio.info.format,
                   msg->audio.info.channels, msg->audio.info.rate,
                   audio_common_dsp::to_nanoseconds(msg->header.stamp.sec,
                                                    msg->header.stamp.nanosec),
                   stream_key);
}

void AudioPlayerNode::write_data(
    const audio_common_msgs::msg::AudioData &input_data, int format,
    int channels, int rate, int64_t stamp_ns, const std::string &stream_key) {

  auto stream_it = this->stream_dict_.find(stream_key);
  if (stream_it == this->stream_dict_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Audio stream not found");
    return;
  }

  if (channels <= 0 || rate <= 0) {
    RCLCPP_WARN(this->get_logger(), "Invalid audio data shape");
    return;
  }

  auto &stream = *stream_it->second;

  std::vector<float> data;
  try {
    data = audio_common_dsp::AudioConverter::to_float(input_data, format,
                                                      channels);
    data = audio_common_dsp::AudioConverter::convert_channels(data, channels,
                                                              this->channels_);
  } catch (const std::invalid_argument &e) {
    RCLCPP_WARN(this->get_logger(), "Dropping audio message: %s", e.what());
    return;
  }

  const size_t input_frames = data.size() / this->channels_;
  if (input_frames == 0) {
    return;
  }

  const float *write_data = data.data();
  size_t total_frames = input_frames;
  std::vector<float> resampled_data;
  std::vector<float> tail_data;

  if (rate != stream.output_rate) {
    // Detect a discontinuity (new audio source or a long pause) and flush the
    // retained frame of the previous block so its state does not bleed into
    // the new audio.
    const double expected_gap =
        static_cast<double>(input_frames) / static_cast<double>(rate);
    const bool discontinuity =
        stream.last_stamp_ns != 0 && stamp_ns > 0 &&
        stamp_ns > stream.last_stamp_ns &&
        static_cast<double>(stamp_ns - stream.last_stamp_ns) / 1e9 >
            std::max(0.5, 2.0 * expected_gap);

    if (discontinuity) {
      tail_data = stream.resampler.flush();
    }

    resampled_data = stream.resampler.process(data, this->channels_, rate,
                                              stream.output_rate);
    write_data = resampled_data.data();
    total_frames = resampled_data.size() / this->channels_;
  }

  if (stamp_ns > 0) {
    stream.last_stamp_ns = stamp_ns;
  }

  if (total_frames == 0 && tail_data.empty()) {
    return;
  }

  const size_t max_samples =
      static_cast<size_t>(stream.output_rate) * this->channels_;

  std::lock_guard<std::mutex> lock(stream.queue_mutex);
  if (!tail_data.empty()) {
    stream.queue.insert(stream.queue.end(), tail_data.begin(), tail_data.end());
  }
  if (total_frames > 0) {
    stream.queue.insert(stream.queue.end(), write_data,
                        write_data + total_frames * this->channels_);
  }

  // Bound latency if the producer runs far ahead of the output device.
  if (stream.queue.size() > max_samples) {
    const size_t excess = stream.queue.size() - max_samples;
    stream.queue.erase(stream.queue.begin(), stream.queue.begin() + excess);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Audio playback queue overflow; dropping old samples");
  }
}

int AudioPlayerNode::playback_callback(
    const void *input, void *output, unsigned long frames,
    const PaStreamCallbackTimeInfo *time_info,
    PaStreamCallbackFlags status_flags, void *user_data) {
  (void)input;
  (void)status_flags;

  auto *stream = static_cast<PlaybackStream *>(user_data);
  auto *out = static_cast<float *>(output);
  const size_t channels = static_cast<size_t>(stream->channels);
  const size_t samples = static_cast<size_t>(frames) * channels;

  std::vector<float> block(samples, 0.0f);
  {
    std::lock_guard<std::mutex> lock(stream->queue_mutex);
    for (size_t i = 0; i < samples; ++i) {
      if (!stream->queue.empty()) {
        block[i] = stream->queue.front();
        stream->queue.pop_front();
      }
    }
  }
  std::copy(block.begin(), block.end(), out);

  if (stream->owner == nullptr || !stream->owner->reference_pub_) {
    return paContinue;
  }

  ReferenceBlock reference;
  reference.samples = std::move(block);
  reference.frames = static_cast<size_t>(frames);

  double latency = 0.0;
  if (time_info != nullptr) {
    latency = time_info->outputBufferDacTime - time_info->currentTime;
  }
  const rclcpp::Time stamp =
      stream->owner->now() + rclcpp::Duration::from_seconds(latency);
  const int64_t stamp_ns = stamp.nanoseconds();
  reference.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
  reference.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);

  {
    std::lock_guard<std::mutex> lock(stream->reference_mutex);
    stream->reference_queue.push_back(std::move(reference));
    while (stream->reference_queue.size() > 32) {
      stream->reference_queue.pop_front();
    }
  }
  stream->reference_cv.notify_one();
  return paContinue;
}

void AudioPlayerNode::reference_loop(PlaybackStream *stream) {
  while (!stream->stop.load()) {
    std::unique_lock<std::mutex> lock(stream->reference_mutex);
    stream->reference_cv.wait(lock, [&]() {
      return stream->stop.load() || !stream->reference_queue.empty();
    });
    if (stream->stop.load()) {
      break;
    }

    ReferenceBlock block = std::move(stream->reference_queue.front());
    stream->reference_queue.pop_front();
    lock.unlock();

    if (!this->reference_pub_ || block.frames == 0) {
      continue;
    }

    auto msg = make_reference_message(block.samples.data(), block.frames,
                                      stream->channels, stream->output_rate,
                                      block.stamp);
    this->reference_pub_->publish(msg);
  }
}
