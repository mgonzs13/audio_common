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


import pyaudio
import numpy as np
from audio_common_msgs.msg import AudioData

pyaudio_to_np = {
    pyaudio.paFloat32: np.float32,
    pyaudio.paInt32: np.int32,
    pyaudio.paInt24: np.int32,
    pyaudio.paInt16: np.int16,
    pyaudio.paInt8: np.int8,
    pyaudio.paUInt8: np.uint8
}


def data_to_array(data: bytes, audio_format: int) -> np.ndarray:

    if audio_format not in pyaudio_to_np:
        return None

    return np.frombuffer(
        data, dtype=pyaudio_to_np[audio_format])


def data_to_msg(data: bytes, audio_format: int) -> AudioData:

    msg = AudioData()
    list_data = data_to_array(data, audio_format).tolist()

    if list_data is None:
        return None

    if audio_format == pyaudio.paFloat32:
        msg.float32_data = list_data
    elif audio_format == pyaudio.paInt32:
        msg.int32_data = list_data
    elif audio_format == pyaudio.paInt24:
        msg.int24_data = list_data
    elif audio_format == pyaudio.paInt16:
        msg.int16_data = list_data
    elif audio_format == pyaudio.paInt8:
        msg.int8_data = list_data
    elif audio_format == pyaudio.paUInt8:
        msg.uint8_data = list_data
    else:
        return None

    return msg


def msg_to_array(msg: AudioData, audio_format: int) -> np.ndarray:

    data = None

    if audio_format == pyaudio.paFloat32:
        data = msg.float32_data
    elif audio_format == pyaudio.paInt32:
        data = msg.int32_data
    elif audio_format == pyaudio.paInt24:
        data = msg.int24_data
    elif audio_format == pyaudio.paInt16:
        data = msg.int16_data
    elif audio_format == pyaudio.paInt8:
        data = msg.int8_data
    elif audio_format == pyaudio.paUInt8:
        data = msg.uint8_data

    if data is not None:
        data = np.frombuffer(data, pyaudio_to_np[audio_format])

    return data
