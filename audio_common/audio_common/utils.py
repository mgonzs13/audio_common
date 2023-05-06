
import pyaudio
import numpy as np
from audio_common_msgs.msg import AudioData
from audio_common_msgs.msg import Audio

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
