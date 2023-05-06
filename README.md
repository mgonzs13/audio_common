# audio_capture

This repositiory provides a set of ROS 2 for audio.

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone --recurse-submodules https://github.com/mgonzs13/audio_common.git
$ pip3 install -r audio_common/requirements.txt
$ cd ~/ros2_ws
$ colcon build
```

## Demo

### Audio Capturer/Player

```shell
$ ros2 run audio_capture audio_capturer
```

```shell
$ ros2 run audio_capture audio_player
```
