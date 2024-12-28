# audio_capture

This repositiory provides a set of ROS 2 packages for audio. It provides a C++ version to capture and play audio data using PortAudio.

[![License: MIT](https://img.shields.io/badge/GitHub-MIT-informational)](https://opensource.org/license/mit) [![GitHub release](https://img.shields.io/github/release/mgonzs13/audio_common.svg)](https://github.com/mgonzs13/audio_common/releases) [![Code Size](https://img.shields.io/github/languages/code-size/mgonzs13/audio_common.svg?branch=main)](https://github.com/mgonzs13/audio_common?branch=main) [![Last Commit](https://img.shields.io/github/last-commit/mgonzs13/audio_common.svg)](https://github.com/mgonzs13/audio_common/commits/main) [![GitHub issues](https://img.shields.io/github/issues/mgonzs13/audio_common)](https://github.com/mgonzs13/audio_common/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/mgonzs13/audio_common)](https://github.com/mgonzs13/audio_common/pulls) [![Contributors](https://img.shields.io/github/contributors/mgonzs13/audio_common.svg)](https://github.com/mgonzs13/audio_common/graphs/contributors) [![C++ Formatter Check](https://github.com/mgonzs13/audio_common/actions/workflows/cpp-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/audio_common/actions/workflows/cpp-formatter.yml?branch=main)

<div align="center">

| ROS 2 Distro |                            Branch                            |                                                                                                          Build status                                                                                                          |                                                                 Docker Image                                                                  | Documentation                                                                                                                                                  |
| :----------: | :----------------------------------------------------------: | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :-------------------------------------------------------------------------------------------------------------------------------------------: | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
|  **Humble**  | [`main`](https://github.com/mgonzs13/audio_common/tree/main) |  [![Humble Build](https://github.com/mgonzs13/audio_common/actions/workflows/humble-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/audio_common/actions/workflows/humble-docker-build.yml?branch=main)   |  [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-humble-blue)](https://hub.docker.com/r/mgons/audio_common/tags?name=humble)  | [![Doxygen Deployment](https://github.com/mgonzs13/audio_common/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/audio_common/) |
|   **Iron**   | [`main`](https://github.com/mgonzs13/audio_common/tree/main) |     [![Iron Build](https://github.com/mgonzs13/audio_common/actions/workflows/iron-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/audio_common/actions/workflows/iron-docker-build.yml?branch=main)      |    [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-iron-blue)](https://hub.docker.com/r/mgons/audio_common/tags?name=iron)    | [![Doxygen Deployment](https://github.com/mgonzs13/audio_common/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/audio_common/) |
|  **Jazzy**   | [`main`](https://github.com/mgonzs13/audio_common/tree/main) |    [![Jazzy Build](https://github.com/mgonzs13/audio_common/actions/workflows/jazzy-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/audio_common/actions/workflows/jazzy-docker-build.yml?branch=main)    |   [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-jazzy-blue)](https://hub.docker.com/r/mgons/audio_common/tags?name=jazzy)   | [![Doxygen Deployment](https://github.com/mgonzs13/audio_common/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/audio_common/) |
| **Rolling**  | [`main`](https://github.com/mgonzs13/audio_common/tree/main) | [![Rolling Build](https://github.com/mgonzs13/audio_common/actions/workflows/rolling-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/audio_common/actions/workflows/rolling-docker-build.yml?branch=main) | [![Docker Image](https://img.shields.io/badge/Docker%20Image%20-rolling-blue)](https://hub.docker.com/r/mgons/audio_common/tags?name=rolling) | [![Doxygen Deployment](https://github.com/mgonzs13/audio_common/actions/workflows/doxygen-deployment.yml/badge.svg)](https://mgonzs13.github.io/audio_common/) |

</div>

## Table of Contents

1. [Installation](#installation)
2. [Docker](#docker)
3. [Nodes](#nodes)
4. [Demos](#demos)

## Installation

```shell
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/audio_common.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install -r audio_common/requirements.txt
colcon build
```

## Docker

You can create a docker image to test audio_common. Use the following common inside the directory of audio_common.

```shell
docker build -t audio_common .
```

After the image is created, run a docker container with the following command.

```shell
docker run -it --device /dev/snd audio_common
```

To use a shortcut, you may use following command:

```shell
make docker_run
```

## Nodes

### audio_capturer_node

Node to obtain audio data from a microphone and publish it into the `audio` topic.

<details>
<summary>Click to expand</summary>

#### Parameters

- **format**: Specifies the audio format to be used for capturing. Common values are `paInt16` (16-bit format) or other formats supported by PortAudio. Default: `paInt16`

- **channels**: The number of audio channels to capture. Typically, `1` for mono and `2` for stereo. Default: `1`

- **rate**: The sample rate that is is how many samples per second should be captured. Default: `16000`

- **chunk**: The size of each audio frames. Default: `4096`

- **device**: The ID of the audio input device. A value of `-1` indicates that the default audio input device should be used. Default: `-1`

- **frame_id**: An identifier for the audio frame. This can be useful for synchronizing audio data with other data streams. Default: `""`

#### ROS 2 Interfaces

- **audio**: Topic to publish the audio data captured from the microphone. Type: `audio_common_msgs/msg/AudioStamped`

</details>

### audio_player_node

Node to play the audio data obtained from the `audio` topic.

<details>
<summary>Click to expand</summary>

#### Parameters

- **channels**: The number of audio channels to capture. Typically, `1` for mono and `2` for stereo. Default: `1`

- **device**: The ID of the audio input device. A value of `-1` indicates that the default audio input device should be used. Default: `-1`

#### ROS 2 Interfaces

- **audio**: Topic subscriber to get the audio data captured to be played. Type: `audio_common_msgs/msg/AudioStamped`

</details>

### music_node

Node to play the music from a audio file in `wav` format.

<details>
<summary>Click to expand</summary>

#### Parameters

- **chunk_time**: Time, in milliseconds, that last each audio chunk. Default: `50`

- **frame_id**: An identifier for the audio frame. This can be useful for synchronizing audio data with other data streams. Default: `""`

#### ROS 2 Interfaces

- **audio**: Topic subscriber to get the audio data captured to be played. Type: `audio_common_msgs/msg/AudioStamped`

</details>

### tts_node

Node to generate audio from a text (TTS).

<details>
<summary>Click to expand</summary>

#### Parameters

- **chunk**: The size of each audio frames. Default: `4096`

- **frame_id**: An identifier for the audio frame. This can be useful for synchronizing audio data with other data streams. Default: `""`

#### ROS 2 Interfaces

- **audio**: Topic publisher to send the audio data generated by the TTS. Type: `audio_common_msgs/msg/AudioStamped`

- **say**: Action to generate audio data from a text. Type: `audio_common_msgs/action/TTS`

</details>

## Demos

### Audio Capturer/Player

```shell
ros2 run audio_common audio_capturer_node
```

```shell
ros2 run audio_common audio_player_node
```

### TTS

```shell
ros2 run audio_common tts_node
```

```shell
ros2 run audio_common audio_player_node
```

```shell
ros2 action send_goal /say audio_common_msgs/action/TTS "{'text': 'Hello World'}"
```

### Music Player

```shell
ros2 run audio_common music_node
```

```shell
ros2 run audio_common audio_player_node
```

```shell
ros2 service call /music_play audio_common_msgs/srv/MusicPlay "{audio: 'elevator'}"
```
