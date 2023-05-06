#!/usr/bin/env python3

import pyaudio
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

        self.audio = pyaudio.PyAudio()
        self.stream_dict = {}

        qos_profile = qos_profile_sensor_data
        qos_profile.depth = 200
        self.sub = self.create_subscription(
            AudioStamped, "audio", self.audio_callback, qos_profile)

        self.player_srv = self.create_service(
            PlayAudio, "play_audio", self.play_audio_srv)

    def destroy_node(self) -> bool:
        self.audio.terminate()

        for key in self.stream_dict:
            self.stream_dict[key].close()

        return super().destroy_node()

    def audio_callback(self, msg: AudioStamped) -> None:

        stream_key = f"{msg.audio.info.format}_{msg.audio.info.rate}"

        if stream_key not in self.stream_dict:
            self.stream_dict[stream_key] = self.audio.open(
                format=msg.audio.info.format,
                channels=self.channels,
                rate=msg.audio.info.rate,
                output=True
            )

        data = [int(ele) for ele in msg.audio.data]
        data = bytes(data)

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
