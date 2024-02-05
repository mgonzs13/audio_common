# audio_capture

This repositiory provides a set of ROS 2 packages for audio.

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone --recurse-submodules https://github.com/mgonzs13/audio_common.git
$ sudo apt install portaudio19-dev
$ pip3 install -r audio_common/requirements.txt
$ cd ~/ros2_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build
```

## Demos

### Audio Capturer/Player

```shell
$ ros2 run audio_common audio_capturer_node
```

```shell
$ ros2 run audio_common audio_player_node
```

### TTS

```shell
$ ros2 run audio_common tts_node
```

```shell
$ ros2 run audio_common audio_player_node
```

```shell
$ ros2 action send_goal /say audio_common_msgs/action/TTS "{'text': 'Hello World'}"
```
