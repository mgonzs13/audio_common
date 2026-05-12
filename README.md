# audio_capture (working)

This repositiory provides a set of ROS 2 packages for audio. It provides a C++ version to capture and play audio data using PortAudio.

Original: [mgonzs13/audio_common](https://github.com/mgonzs13/audio_common)

## Installation

```shell
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/audio_common.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Run (NUC)
```shell
ros2 run audio_common audio_capturer_node

ros2 lifecycle set /audio_capturer_node configure

ros2 lifecycle set /audio_capturer_node activate

```

## Run (Operator)
```shell
ros2 run audio_common audio_player_node

ros2 lifecycle set /audio_player_node configure

ros2 lifecycle set /audio_player_node activate

```

## Docs
[audio_common for ROS2](https://mgonzs13.github.io/audio_common/4.0.8/index.html)