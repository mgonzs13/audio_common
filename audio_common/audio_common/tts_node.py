#!/usr/bin/env python3


import os
import wave
import tempfile
import threading
import pyaudio
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from audio_common_msgs.msg import AudioStamped
from audio_common_msgs.srv import PlayAudio
from audio_common_msgs.action import TTS


class AudioCapturerNode(Node):

    def __init__(self) -> None:
        super().__init__("tts_node")

        self.declare_parameters("", [
            ("chunk", 4096),
            ("frame_id", "")
        ])

        self.chunk = self.get_parameter(
            "chunk").get_parameter_value().integer_value
        self.frame_id = self.get_parameter(
            "frame_id").get_parameter_value().string_value

        self.espeak_cmd = "espeak -v{} -s{} -a{} -w {} '{}'"

        self.player_client = self.create_client(
            PlayAudio, "play_audio", callback_group=ReentrantCallbackGroup())

        # action server
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            TTS,
            "tts",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

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

        pyaudio_to_np = {
            pyaudio.paFloat32: np.float32,
            pyaudio.paInt32: np.int32,
            pyaudio.paInt24: np.int32,
            pyaudio.paInt16: np.int16,
            pyaudio.paInt8: np.int8,
            pyaudio.paUInt8: np.uint8
        }

        text = goal_handle.request.text
        language = goal_handle.request.language
        rate = goal_handle.request.rate * 350
        volume = goal_handle.request.volume * 200

        # create espeak audio file
        audio_file = tempfile.NamedTemporaryFile(mode="w+")
        os.system(self.espeak_cmd.format(
            language, rate, volume, audio_file.name, text))

        # pub audio
        audio_file.seek(0)
        wf = wave.open(audio_file.name, "rb")
        audio_file.close()
        audio_format = pyaudio.get_format_from_width(wf.getsampwidth())

        data = wf.readframes(self.chunk)
        while data:
            if not goal_handle.is_active:
                return TTS.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return TTS.Result()

            data = np.frombuffer(
                data, dtype=pyaudio_to_np[audio_format]).tolist()

            msg = AudioStamped()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = self.get_clock().now().to_msg()

            if audio_format == pyaudio.paFloat32:
                msg.audio.audio_data.float32_data = data
            elif audio_format == pyaudio.paInt32:
                msg.audio.audio_data.int32_data = data
            elif audio_format == pyaudio.paInt24:
                msg.audio.audio_data.int24_data = data
            elif audio_format == pyaudio.paInt16:
                msg.audio.audio_data.int16_data = data
            elif audio_format == pyaudio.paInt8:
                msg.audio.audio_data.int8_data = data
            elif audio_format == pyaudio.paUInt8:
                msg.audio.audio_data.uint8_data = data

            msg.audio.info.format = pyaudio.get_format_from_width(
                wf.getsampwidth())
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
