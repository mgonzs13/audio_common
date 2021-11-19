import pyaudio

CHUNK = 1024*4
CHANNELS = 1
RATE = 44100


p = pyaudio.PyAudio()

stream = p.open(format=pyaudio.paInt16,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                output=True,
                frames_per_buffer=CHUNK)

print("* recording")

while 1:
    data = stream.read(CHUNK)
    #data = [int(ele) for ele in data]
    #data = bytes(data)
    data = str(data).encode("utf8")
    print(len(data), type(data), "\n", data, "\n"*5)
    stream.write(data, CHUNK)

print("* done")

stream.stop_stream()
stream.close()

p.terminate()
