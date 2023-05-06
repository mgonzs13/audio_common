#!/usr/bin/env python3

import pyaudio
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from audio_common_msgs.msg import AudioDataStamped


class AudioPlayerNode(Node):

    def __init__(self):
        super().__init__("audio_player_node")

        self.declare_parameters("", [
            ("format", pyaudio.paInt16),
            ("channels", 1),
            ("rate", 16000),
            ("chunk", 4096)
        ])

        self.format = self.get_parameter(
            "format").get_parameter_value().integer_value
        self.channels = self.get_parameter(
            "channels").get_parameter_value().integer_value
        self.rate = self.get_parameter(
            "rate").get_parameter_value().integer_value
        self.chunk = self.get_parameter(
            "chunk").get_parameter_value().integer_value

        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(format=self.format,
                                      channels=self.channels,
                                      rate=self.rate,
                                      output=True,
                                      frames_per_buffer=self.chunk)

        self.sub = self.create_subscription(
            AudioDataStamped, "audio", self.audio_callback, qos_profile_sensor_data)

    def destroy_node(self) -> bool:
        self.stream.close()
        self.audio.terminate()
        return super().destroy_node()

    def audio_callback(self, msg: AudioDataStamped):
        data = [int(ele) for ele in msg.audio.data]
        data = bytes(data)
        self.stream.write(data, self.chunk)


def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
