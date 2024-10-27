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

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from audio_common_msgs.msg import AudioStamped
from audio_common.utils import data_to_msg


class AudioCapturerNode(Node):

    def __init__(self) -> None:
        super().__init__("audio_capturer_node")

        self.declare_parameters(
            "",
            [
                ("format", pyaudio.paInt16),
                ("channels", 1),
                ("rate", 16000),
                ("chunk", 4096),
                ("device", -1),
                ("frame_id", ""),
            ],
        )

        self.format = self.get_parameter("format").get_parameter_value().integer_value
        self.channels = (
            self.get_parameter("channels").get_parameter_value().integer_value
        )
        self.rate = self.get_parameter("rate").get_parameter_value().integer_value
        self.chunk = self.get_parameter("chunk").get_parameter_value().integer_value
        device = self.get_parameter("device").get_parameter_value().integer_value
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )

        if device < 0:
            device = None

        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk,
            input_device_index=device,
        )

        self.audio_pub = self.create_publisher(
            AudioStamped, "audio", qos_profile_sensor_data
        )

        self.get_logger().info("AudioCapturer node started")

    def destroy_node(self) -> bool:
        self.stream.close()
        self.audio.terminate()
        return super().destroy_node()

    def work(self) -> None:

        while rclpy.ok:
            data = self.stream.read(self.chunk)

            msg = AudioStamped()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = self.get_clock().now().to_msg()

            audio_msg = data_to_msg(data, self.format)
            if audio_msg is None:
                self.get_logger().error(f"Format {self.format} unknown")
                return

            msg.audio = audio_msg
            msg.audio.info.channels = self.channels
            msg.audio.info.chunk = self.chunk
            msg.audio.info.rate = self.rate

            self.audio_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AudioCapturerNode()
    node.work()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
