#!/usr/bin/env python3

# MIT License

# Copyright (c) 2023  Miguel Ángel González Santamarta
# Copyright (c) 2024  Alejandro González Cantón

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


import os
import pyaudio
from pydub import AudioSegment
from pydub.utils import make_chunks
from pydub.utils import mediainfo_json

from threading import Event
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

from audio_common.utils import data_to_msg
from std_srvs.srv import Trigger
from audio_common_msgs.srv import MusicPlay
from audio_common_msgs.msg import AudioStamped


class MusicNode(Node):

    def __init__(self) -> None:
        super().__init__("music_node")

        # parameters
        self.declare_parameters("", [
            ("chunk_time", 50),
            ("frame_id", ""),
        ])

        self.chunk_time = self.get_parameter(
            "chunk_time").get_parameter_value().integer_value
        self.frame_id = self.get_parameter(
            "frame_id").get_parameter_value().string_value

        # audio pub
        self.publish_thread = None
        self.music_event = Event()
        self.pause_music = False
        self.stop_music = False
        self.audio_loop = False

        self.player_pub = self.create_publisher(
            AudioStamped, "audio", qos_profile_sensor_data)

        # services
        self.play_service = self.create_service(
            MusicPlay, "music_play", self.play_callback)

        self.stop_service = self.create_service(
            Trigger, "music_stop", self.stop_callback)

        self.pause_service = self.create_service(
            Trigger, "music_pause", self.pause_callback)

        self.resume_service = self.create_service(
            Trigger, "music_resume", self.resume_callback)

        self.get_logger().info("Music node started")

    def publish_audio(self, audio: AudioSegment) -> None:
        audio_data = make_chunks(audio, self.chunk_time)
        audio_format = pyaudio.get_format_from_width(audio.sample_width)
        chunk_size = round(self.chunk_time * audio.frame_rate / 1000)

        pub_rate = self.create_rate(1000 / self.chunk_time)

        while not self.stop_music:
            for chunk in audio_data:

                audio_msg = data_to_msg(chunk.raw_data, audio_format)
                if audio_msg is None:
                    self.get_logger().error(f"Format {audio_format} unknown")
                    return

                msg = AudioStamped()
                msg.header.frame_id = self.frame_id
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.audio = audio_msg
                msg.audio.info.channels = audio.channels
                msg.audio.info.chunk = chunk_size
                msg.audio.info.rate = audio.frame_rate

                self.player_pub.publish(msg)
                pub_rate.sleep()

                if self.pause_music:
                    self.music_event.clear()
                    self.music_event.wait()

                if self.stop_music:
                    break

            if not self.audio_loop and not self.stop_music:
                break

    def play_callback(
        self,
        request: MusicPlay.Request,
        response: MusicPlay.Response
    ) -> MusicPlay.Response:

        if self.publish_thread and self.publish_thread.is_alive():
            self.get_logger().warn("There is other music playing")
            response.success = False
            return response

        path = request.file_path

        if not path:
            path = os.path.join(
                get_package_share_directory(
                    "audio_common"), "samples", request.audio + ".mp3")

        if not os.path.exists(path):
            self.get_logger().error(f"File {path} not found")
            response.success = False

        else:
            music_data: AudioSegment = AudioSegment.from_file(path)

            self.get_logger().info(f"Playing {path}")

            try:
                self.get_logger().info(
                    f"Title: {mediainfo_json(path)['format']['tags']['title']}")
            except KeyError:
                pass

            self.get_logger().info(
                f"Duration: {music_data.duration_seconds} seconds")

            self.audio_loop = request.loop
            self.pause_music = False
            self.stop_music = False
            response.success = True

            self.publish_thread = Thread(
                target=self.publish_audio, args=(music_data,))
            self.publish_thread.start()

        return response

    def pause_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response
    ) -> Trigger.Response:
        if self.publish_thread and self.publish_thread.is_alive():
            self.pause_music = True
            self.get_logger().info("Music paused")
            response.success = True
        else:
            self.get_logger().warn("No music to pause")
            response.success = False

        return response

    def resume_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response
    ) -> Trigger.Response:
        if self.publish_thread and self.publish_thread.is_alive():
            self.pause_music = False
            self.music_event.set()
            self.get_logger().info("Music resumed")
            response.success = True
        else:
            self.get_logger().warn("No music to resume")
            response.success = False

        return response

    def stop_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response
    ) -> Trigger.Response:
        if self.publish_thread and self.publish_thread.is_alive():
            self.stop_music = True
            self.publish_thread.join()
            self.get_logger().info("Music stopped")
            response.success = True

        else:
            self.get_logger().warn("No music to stop")
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MusicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
