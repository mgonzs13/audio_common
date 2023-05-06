#!/usr/bin/env python3

import pyaudio
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from audio_common_msgs.msg import AudioStamped
from audio_common_msgs.srv import PlayAudio


class AudioPlayerNode(Node):

    def __init__(self) -> None:
        super().__init__("audio_player_node")

        self.declare_parameter("channels", 1)
        self.channels = self.get_parameter(
            "channels").get_parameter_value().integer_value

        self.declare_parameter("device", -1)
        self.device = self.get_parameter(
            "device").get_parameter_value().integer_value

        if self.device < 0:
            self.device = None

        self.audio = pyaudio.PyAudio()
        self.stream_dict = {}

        self.sub = self.create_subscription(
            AudioStamped, "audio", self.audio_callback, qos_profile_sensor_data)

        self.player_srv = self.create_service(
            PlayAudio, "play_audio", self.play_audio_srv)

    def destroy_node(self) -> bool:
        self.audio.terminate()

        for key in self.stream_dict:
            self.stream_dict[key].close()

        return super().destroy_node()

    def audio_callback(self, msg: AudioStamped) -> None:

        stream_key = f"{msg.audio.info.format}_{msg.audio.info.rate}"
        pyaudio_to_np = {
            pyaudio.paFloat32: np.float32,
            pyaudio.paInt32: np.int32,
            pyaudio.paInt24: np.int32,
            pyaudio.paInt16: np.int16,
            pyaudio.paInt8: np.int8,
            pyaudio.paUInt8: np.uint8
        }

        if stream_key not in self.stream_dict:
            self.stream_dict[stream_key] = self.audio.open(
                format=msg.audio.info.format,
                channels=self.channels,
                rate=msg.audio.info.rate,
                output=True,
                output_device_index=self.device
            )

        data = None

        if msg.audio.info.format == pyaudio.paFloat32:
            data = msg.audio.audio_data.float32_data
        elif msg.audio.info.format == pyaudio.paInt32:
            data = msg.audio.audio_data.int32_data
        elif msg.audio.info.format == pyaudio.paInt24:
            data = msg.audio.audio_data.int24_data
        elif msg.audio.info.format == pyaudio.paInt16:
            data = msg.audio.audio_data.int16_data
        elif msg.audio.info.format == pyaudio.paInt8:
            data = msg.audio.audio_data.int8_data
        elif msg.audio.info.format == pyaudio.paUInt8:
            data = msg.audio.audio_data.uint8_data
        else:
            self.get_logger().error(f"Format {msg.audio.info.format} unknown")
            return

        data = np.frombuffer(data, pyaudio_to_np[msg.audio.info.format])

        if msg.audio.info.channels != self.channels:
            if msg.audio.info.channels == 1 and self.channels == 2:
                # mono to stereo
                data = np.repeat(data, 2)

            elif msg.audio.info.channels == 2 and self.channels == 1:
                # stereo to mono
                data = np.mean(data.reshape(-1, 2), axis=1).astype(int)

        data = data.tobytes()

        stream = self.stream_dict[stream_key]
        stream.write(data)

    def play_audio_srv(
        self,
            req: PlayAudio.Request,
            res: PlayAudio.Response
    ) -> PlayAudio.Response:
        self.audio_callback(req.audio)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
