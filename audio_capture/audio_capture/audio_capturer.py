#!/usr/bin/env python3

import pyaudio
import time
from typing import List, Mapping, Tuple, Optional

import rclpy
from rclpy.node import Node
from audio_capture_interfaces.msg import Audio


class AudioCapturerNode(Node):

    def __init__(self):
        super().__init__("audio_capturer_node")

        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        self.chunk = 4096
        self.audio = pyaudio.PyAudio()

        self.stream = self.audio.open(format=self.format,
                                      channels=self.channels,
                                      rate=self.rate,
                                      input=True,
                                      frames_per_buffer=self.chunk)

        self.pub = self.create_publisher(Audio, "audio", 1)

    def destroy_node(self) -> bool:
        self.stream.close()
        self.audio.terminate()
        return super().destroy_node()

    def capture(self):

        while rclpy.ok():
            data = self.stream.read(self.chunk)

            msg = Audio()

            msg.data = list(data)
            msg.channels = self.channels
            msg.format = self.format
            msg.chunk = self.chunk

            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AudioCapturerNode()
    node.capture()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
