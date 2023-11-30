#!/usr/bin/env python3

# MIT License

# Copyright (c) 2023  Miguel Ángel González Santamarta

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
import wave
import pyaudio
import tempfile
import threading
from TTS.api import TTS as TtsModel

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from audio_common_msgs.msg import AudioStamped
from audio_common_msgs.srv import PlayAudio
from audio_common_msgs.action import TTS
from audio_common.utils import data_to_msg


class AudioCapturerNode(Node):

    def __init__(self) -> None:
        super().__init__("tts_node")

        self.declare_parameters("", [
            ("chunk", 4096),
            ("frame_id", ""),

            ("model", "tts_models/en/ljspeech/vits"),
            ("speaker_wav", ""),
            ("device", "cpu")
        ])

        self.chunk = self.get_parameter(
            "chunk").get_parameter_value().integer_value
        self.frame_id = self.get_parameter(
            "frame_id").get_parameter_value().string_value

        self.model = self.get_parameter(
            "model").get_parameter_value().string_value
        self.speaker_wav = self.get_parameter(
            "speaker_wav").get_parameter_value().string_value
        self.device = self.get_parameter(
            "device").get_parameter_value().string_value

        if (
            not self.speaker_wav or
            not (
                os.path.exists(self.speaker_wav) and
                os.path.isfile(self.speaker_wav)
            )
        ):
            self.speaker_wav = None

        self.tts = TtsModel(self.model).to(self.device)

        self.player_client = self.create_client(
            PlayAudio, "play_audio", callback_group=ReentrantCallbackGroup())

        # action server
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            TTS,
            "say",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info("TTS node started")

    def destroy_node(self) -> bool:
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request: ServerGoalHandle) -> int:
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle) -> None:
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> None:
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle) -> TTS.Result:

        request: TTS.Goal = goal_handle.request

        text = request.text
        language = request.language
        speed = request.speed
        emotion = request.emotion

        if not self.tts.is_coqui_studio:
            speed = None
            emotion = None

        if not self.tts.is_multi_lingual:
            language = None

        # create audio file
        audio_file = tempfile.NamedTemporaryFile(mode="w+")
        self.tts.tts_to_file(
            text,
            speaker_wav=self.speaker_wav,
            language=language,
            speed=speed,
            emotion=emotion,
            file_path=audio_file.name
        )

        # pub audio
        audio_file.seek(0)
        wf = wave.open(audio_file.name, "rb")
        audio_file.close()
        audio_format = pyaudio.get_format_from_width(wf.getsampwidth())

        # send audio data
        data = wf.readframes(self.chunk)
        while data:
            if not goal_handle.is_active:
                return TTS.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return TTS.Result()

            audio_data_msg = data_to_msg(data, audio_format)
            if audio_data_msg is None:
                self.get_logger().error(f"Format {audio_format} unknown")
                self._goal_handle.abort()
                return TTS.Result()

            msg = AudioStamped()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.audio.audio_data = audio_data_msg
            msg.audio.info.format = audio_format
            msg.audio.info.channels = wf.getnchannels()
            msg.audio.info.chunk = self.chunk
            msg.audio.info.rate = wf.getframerate()

            request = PlayAudio.Request()
            request.audio = msg
            self.player_client.wait_for_service()
            self.player_client.call(request)

            data = wf.readframes(self.chunk)

        goal_handle.succeed()
        return TTS.Result()


def main(args=None):
    rclpy.init(args=args)
    node = AudioCapturerNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
