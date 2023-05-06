#!/usr/bin/env python3

import pyaudio
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from audio_common_msgs.msg import AudioStamped


class AudioPlayerNode(Node):

    def __init__(self):
        super().__init__("audio_player_node")

        self.declare_parameters("", [
            ("channels", 1)
        ])

        self.channels = self.get_parameter(
            "channels").get_parameter_value().integer_value

        self.audio = pyaudio.PyAudio()
        self.stream_dict = {}

        self.sub = self.create_subscription(
            AudioStamped, "audio", self.audio_callback, qos_profile_sensor_data)

    def destroy_node(self) -> bool:
        self.audio.terminate()

        for key in self.stream_dict:
            self.stream_dict[key].close()

        return super().destroy_node()

    def audio_callback(self, msg: AudioStamped):

        stream_key = f"{msg.audio.info.format}_{msg.audio.info.rate}"

        if stream_key not in self.stream_dict:
            self.stream_dict[stream_key] = self.audio.open(
                format=msg.audio.info.format,
                channels=self.channels,
                rate=msg.audio.info.rate,
                output=True
            )

        stream = self.stream_dict[stream_key]

        data = [int(ele) for ele in msg.audio.data]
        data = bytes(data)

        stream.write(data)


def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
