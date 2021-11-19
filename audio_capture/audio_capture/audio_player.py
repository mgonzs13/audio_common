#!/usr/bin/env python3

import pyaudio
import rclpy
from rclpy.node import Node
from audio_capture_interfaces.msg import Audio


class AudioPlayerNode(Node):

    def __init__(self):
        super().__init__("audio_player_node")

        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        self.chunk = 4096
        self.audio = pyaudio.PyAudio()

        self.stream = self.audio.open(format=self.format,
                                      channels=self.channels,
                                      rate=self.rate,
                                      output=True,
                                      frames_per_buffer=self.chunk)

        self.sub = self.create_subscription(
            Audio, "audio", self.audio_callback, 1)

    def destroy_node(self) -> bool:
        self.stream.close()
        self.audio.terminate()
        return super().destroy_node()

    def audio_callback(self, msg: Audio):

        data = [int(ele) for ele in msg.data]
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
