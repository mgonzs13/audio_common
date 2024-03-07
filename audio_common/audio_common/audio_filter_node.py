#!/usr/bin/env python3

# MIT License

# Copyright (c) 2024  Miguel Ángel González Santamarta

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


import numpy as np
from scipy.signal import butter, sosfiltfilt

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from audio_common_msgs.msg import AudioStamped
from audio_common.utils import array_to_msg, msg_to_array


class AudioFilterNode(Node):

    def __init__(self) -> None:
        super().__init__("audio_filter_node")

        self.declare_parameter("filter_type", "band")
        self.filter_type = self.get_parameter(
            "filter_type").get_parameter_value().string_value

        self.declare_parameter("low_cutoff", 200)
        self.low_cutoff = self.get_parameter(
            "low_cutoff").get_parameter_value().integer_value

        self.declare_parameter("high_cutoff", 2000)
        self.high_cutoff = self.get_parameter(
            "high_cutoff").get_parameter_value().integer_value

        self.declare_parameter("filter_order", 4)
        self.filter_order = self.get_parameter(
            "filter_order").get_parameter_value().integer_value

        self.audio_pub = self.create_publisher(
            AudioStamped, "filtered_audio", qos_profile_sensor_data)
        self.sub = self.create_subscription(
            AudioStamped, "audio", self.audio_callback, qos_profile_sensor_data)

    def audio_filter(self, data, rate: int) -> np.ndarray:

        Wn = self.low_cutoff

        nyq = 0.5 * rate
        low = self.low_cutoff / nyq
        high = self.high_cutoff / nyq

        if self.filter_type == "high":
            Wn = low
        elif self.filter_type == "low":
            Wn = high
        elif self.filter_type == "band" or self.filter_type == "bandstop":
            Wn = [low, high]

        sos = butter(
            self.filter_order,
            Wn,
            btype=self.filter_type,
            analog=False,
            output="sos"
        )

        filtered_data = sosfiltfilt(sos, data)
        return filtered_data

    def audio_callback(self, msg: AudioStamped) -> None:
        array = msg_to_array(msg.audio.audio_data, msg.audio.info.format)

        if array is None:
            return

        array_filtered = self.audio_filter(array, msg.audio.info.rate)
        msg.audio.audio_data = array_to_msg(
            array_filtered, msg.audio.info.format)
        self.audio_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AudioFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
