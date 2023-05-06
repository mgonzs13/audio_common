#!/usr/bin/env python3

import pyaudio
import numpy as np

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
            ("device", -1),
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
        device = self.get_parameter(
            "device").get_parameter_value().integer_value
        self.frame_id = self.get_parameter(
            "frame_id").get_parameter_value().string_value

        if device < 0:
            device = None

        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk,
            input_device_index=device
        )

        self.audio_pub = self.create_publisher(
            AudioStamped, "audio", qos_profile_sensor_data)

    def destroy_node(self) -> bool:
        self.stream.close()
        self.audio.terminate()
        return super().destroy_node()

    def work(self) -> None:

        pyaudio_to_np = {
            pyaudio.paFloat32: np.float32,
            pyaudio.paInt32: np.int32,
            pyaudio.paInt24: np.int32,
            pyaudio.paInt16: np.int16,
            pyaudio.paInt8: np.int8,
            pyaudio.paUInt8: np.uint8
        }

        while rclpy.ok:
            data = self.stream.read(self.chunk)
            data = np.frombuffer(
                data, dtype=pyaudio_to_np[self.format]).tolist()

            msg = AudioStamped()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = self.get_clock().now().to_msg()

            if self.format == pyaudio.paFloat32:
                msg.audio.audio_data.float32_data = data
            elif self.format == pyaudio.paInt32:
                msg.audio.audio_data.int32_data = data
            elif self.format == pyaudio.paInt24:
                msg.audio.audio_data.int24_data = data
            elif self.format == pyaudio.paInt16:
                msg.audio.audio_data.int16_data = data
            elif self.format == pyaudio.paInt8:
                msg.audio.audio_data.int8_data = data
            elif self.format == pyaudio.paUInt8:
                msg.audio.audio_data.uint8_data = data
            else:
                self.get_logger().error(f"Format {self.format} unknown")
                return

            msg.audio.info.format = self.format
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
