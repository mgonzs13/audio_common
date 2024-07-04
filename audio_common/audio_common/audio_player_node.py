#!/usr/bin/env python3

# MIT License

# Copyright (c) 2023  Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import pyaudio
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from audio_common_msgs.msg import AudioStamped
from audio_common.utils import msg_to_array, array_to_data, pyaudio_to_np


class AudioPlayerNode(Node):

    def __init__(self) -> None:
        super().__init__("audio_player_node")

        self.declare_parameters("", [
            ("channels", 2),
            ("device", -1),
        ])

        self.channels = self.get_parameter(
            "channels").get_parameter_value().integer_value
        self.device = self.get_parameter(
            "device").get_parameter_value().integer_value

        if self.device < 0:
            self.device = None

        self.audio = pyaudio.PyAudio()
        self.stream_dict = {}

        qos_profile = qos_profile_sensor_data
        self.sub = self.create_subscription(
            AudioStamped, "audio", self.audio_callback, qos_profile)

        self.get_logger().info("AudioPlayer node started")

    def destroy_node(self) -> bool:
        for key in self.stream_dict:
            self.stream_dict[key].close()
        self.audio.terminate()
        return super().destroy_node()

    def audio_callback(self, msg: AudioStamped) -> None:

        stream_key = f"{msg.audio.info.format}_{msg.audio.info.rate}"

        if stream_key not in self.stream_dict:
            self.stream_dict[stream_key] = self.audio.open(
                format=msg.audio.info.format,
                channels=self.channels,
                rate=msg.audio.info.rate,
                output=True,
                output_device_index=self.device
            )

        array_data = msg_to_array(msg.audio)
        if array_data is None:
            self.get_logger().error(f"Format {msg.audio.info.format} unknown")
            return

        if msg.audio.info.channels != self.channels:
            if msg.audio.info.channels == 1 and self.channels == 2:
                # mono to stereo
                array_data = np.repeat(array_data, 2)

            elif msg.audio.info.channels == 2 and self.channels == 1:
                # stereo to mono
                array_data = np.mean(array_data.reshape(-1, 2), axis=1)
                array_data = array_data.astype(
                    pyaudio_to_np[msg.audio.info.format])

        data = array_to_data(array_data)
        stream: pyaudio.PyAudio.Stream = self.stream_dict[stream_key]
        stream.write(
            data,
            exception_on_underflow=False,
            num_frames=msg.audio.info.chunk
        )


def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
