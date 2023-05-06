#!/usr/bin/env python3

import pyaudio
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from audio_common_msgs.msg import AudioStamped


class AudioCapturerNode(Node):

    def __init__(self) -> None:
        super().__init__("audio_capturer_node")

        self.declare_parameters("", [
            ("format", pyaudio.paInt16),
            ("channels", 1),
            ("rate", 16000),
            ("chunk", 4096),
            ("frame_id", "")
        ])

        self.format = self.get_parameter(
            "format").get_parameter_value().integer_value
        self.channels = self.get_parameter(
            "channels").get_parameter_value().integer_value
        self.rate = self.get_parameter(
            "rate").get_parameter_value().integer_value
        self.chunk = self.get_parameter(
            "chunk").get_parameter_value().integer_value
        self.frame_id = self.get_parameter(
            "frame_id").get_parameter_value().string_value

        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(format=self.format,
                                      channels=self.channels,
                                      rate=self.rate,
                                      input=True,
                                      frames_per_buffer=self.chunk)

        qos_profile = qos_profile_sensor_data
        qos_profile.depth = 200
        self.audio_pub = self.create_publisher(
            AudioStamped, "audio", qos_profile)
        self.create_timer(self.chunk / self.rate, self.pyaudio_cb)

    def destroy_node(self) -> bool:
        self.stream.close()
        self.audio.terminate()
        return super().destroy_node()

    def pyaudio_cb(self) -> None:

        data = self.stream.read(self.chunk)

        msg = AudioStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.audio.data = list(data)

        msg.audio.info.format = self.format
        msg.audio.info.channels = self.channels
        msg.audio.info.chunk = self.chunk
        msg.audio.info.rate = self.rate

        self.audio_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AudioCapturerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
