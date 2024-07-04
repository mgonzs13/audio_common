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

from threading import Thread
from threading import Event
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

from audio_common_msgs.msg import AudioStamped
from audio_common.utils import data_to_msg

from std_srvs.srv import Trigger
from audio_common_msgs.srv import MusicPlay


class MusicNode(Node):

    def __init__(self) -> None:
        super().__init__("music_node")

        self.declare_parameters("", [
            ("chunk_time", 50),
            ("frame_id", ""),
        ])

        self.chunk_time = self.get_parameter(
            "chunk_time").get_parameter_value().integer_value
        self.frame_id = self.get_parameter(
            "frame_id").get_parameter_value().string_value

        self.player_pub = self.create_publisher(
            AudioStamped, "audio", qos_profile_sensor_data)

        self.play_service = self.create_service(
            MusicPlay, "music_play", self.play_callback)

        self.stop_service = self.create_service(
            Trigger, "music_stop", self.stop_callback)

        self.pause_service = self.create_service(
            Trigger, "music_pause", self.pause_callback)

        self.resume_service = self.create_service(
            Trigger, "music_resume", self.resume_callback)

        self.publish_thread = Thread(target=self.publish_audio)
        self.music_event = Event()
        self.music_data: Optional[AudioSegment] = None
        self.audio_loop = False

        self.publish_thread.start()

        self.get_logger().info("Music node started")

    def publish_audio(self) -> None:
        while True and rclpy.ok():
            if self.music_data is None:
                self.music_event.clear()
                self.music_event.wait()

            audio: AudioSegment = self.music_data
            audio_data = make_chunks(audio, self.chunk_time)
            audio_format = pyaudio.get_format_from_width(audio.sample_width)

            pub_rate = self.create_rate(1000 / self.chunk_time)
            chunk_size = round(self.chunk_time * audio.frame_rate / 1000)

            for chunk in audio_data:
                self.music_event.wait()
                if self.music_data is None:
                    break

                audio_msg = data_to_msg(chunk.raw_data, audio_format)
                if audio_msg is None:
                    self.get_logger().error(f"Format {audio_format} unknown")

                msg = AudioStamped()
                msg.header.frame_id = self.frame_id
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.audio = audio_msg
                msg.audio.info.channels = audio.channels
                msg.audio.info.chunk = chunk_size
                msg.audio.info.rate = audio.frame_rate

                self.player_pub.publish(msg)
                pub_rate.sleep()

            if not self.audio_loop:
                self.music_data = None

    def play_callback(
        self,
        request: MusicPlay.Request,
        response: MusicPlay.Response
    ) -> MusicPlay.Response:

        path = request.file_path

        if not path:
            path = os.path.join(
                get_package_share_directory(
                    "audio_common"), "samples", request.audio + ".mp3")

        if not os.path.exists(path):
            self.get_logger().error(f"File {path} not found")
            response.success = False

        else:
            self.music_data: AudioSegment = AudioSegment.from_file(path)
            self.get_logger().info(f"Playing {path}")
            try:
                self.get_logger().info(
                    f"Title: {mediainfo_json(path)['format']['tags']['title']}")
            except KeyError:
                pass

            self.get_logger().info(
                f"Duration: {self.music_data.duration_seconds} seconds")
            self.audio_loop = request.loop
            self.music_event.set()
            response.success = True

        return response

    def pause_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response
    ) -> Trigger.Response:
        if self.music_data is not None:
            self.music_event.clear()
            self.get_logger().info("Music paused")
            response.success = True
        else:
            self.get_logger().info("No music to pause")
            response.success = False

        return response

    def resume_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response
    ) -> Trigger.Response:
        if self.music_data is not None:
            self.music_event.set()
            self.get_logger().info("Music resumed")
            response.success = True
        else:
            self.get_logger().info("No music to resume")
            response.success = False

        return response

    def stop_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response
    ) -> Trigger.Response:
        if self.music_data is not None:
            self.music_data = None
            self.get_logger().info("Music stopped")
            response.success = True
        else:
            self.get_logger().info("No music to stop")
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
