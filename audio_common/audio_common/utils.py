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
from audio_common_msgs.msg import Audio

pyaudio_to_np = {
    pyaudio.paFloat32: np.float32,
    pyaudio.paInt32: np.int32,
    pyaudio.paInt16: np.int16,
    pyaudio.paInt8: np.int8,
    pyaudio.paUInt8: np.uint8
}

np_to_pyaudio = {
    np.float32: pyaudio.paFloat32,
    np.int32: pyaudio.paInt32,
    np.int16: pyaudio.paInt16,
    np.int8: pyaudio.paInt8,
    np.uint8: pyaudio.paUInt8,
}


def data_to_array(data: bytes, audio_format: int) -> np.ndarray:

    if audio_format not in pyaudio_to_np:
        return None

    return np.frombuffer(
        data, dtype=pyaudio_to_np[audio_format])


def array_to_data(array: np.ndarray) -> bytes:
    return array.tobytes()


def data_to_msg(data: bytes, audio_format: int) -> Audio:

    array = data_to_array(data, audio_format)
    return array_to_msg(array)


def msg_to_data(msg: Audio) -> bytes:

    array = msg_to_array(msg)
    return array.tobytes()


def array_to_msg(array: np.ndarray) -> Audio:

    msg = Audio()
    list_data = array.tolist()

    if list_data is None:
        return None

    audio_format = type(array[0])
    msg.info.format = np_to_pyaudio[audio_format]

    if audio_format == np.float32:
        msg.audio_data.float32_data = list_data
    elif audio_format == np.int32:
        msg.audio_data.int32_data = list_data
    elif audio_format == np.int16:
        msg.audio_data.int16_data = list_data
    elif audio_format == np.int8:
        msg.audio_data.int8_data = list_data
    elif audio_format == np.uint8:
        msg.audio_data.uint8_data = list_data
    else:
        return None

    return msg


def msg_to_array(msg: Audio) -> np.ndarray:

    data = None
    audio_format = msg.info.format

    if audio_format == pyaudio.paFloat32:
        data = msg.audio_data.float32_data
    elif audio_format == pyaudio.paInt32:
        data = msg.audio_data.int32_data
    elif audio_format == pyaudio.paInt16:
        data = msg.audio_data.int16_data
    elif audio_format == pyaudio.paInt8:
        data = msg.audio_data.int8_data
    elif audio_format == pyaudio.paUInt8:
        data = msg.audio_data.uint8_data

    if data is not None:
        data = np.frombuffer(data, pyaudio_to_np[audio_format])

    return data
