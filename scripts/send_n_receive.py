"""
This python program uses pyaudio to play and record audio.

Input audio is selected from one of the three sources: an input .wav file, a tone generator or from recorder.
Received audio is saved in a file, if a name provided or discarded.

"""

import sys
import argparse, readline
import pyaudio
import wave
from datetime import datetime, timedelta
import struct
import numpy as np
import signal
from threading import Event
from enum import Enum
from siggen import Signal

waitThread = Event()

CHUNK = 256
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000

sources = Enum('sources',['NULL','LOOPBACK','INTERNAL','FILE'])
dest    = Enum('dest',['NULL','FILE'])
#SOURCE = 0
#LOOPBACK = 1
#INTERNAL = 2


parser = argparse.ArgumentParser( description='''\
	 A record and play program using pyaudio. 
	Input audio is selected from one of the three sources: 
	  (1) an input .wav file
	  (2) a tone generator or 
	  (3) from the recorder. 
	Received audio is saved in a file, if a name is provided.
	''')

parser.add_argument('devices',nargs='+', help='name of the devices e.g., \"builtin-mic builtin-speaker\"; if a recording device is required for the configurarion, it must appear first in the list')
parser.add_argument('--list_devices',action='store_true',help="lists the audio devices and exits")
parser.add_argument('-if','--input_filename', help='Source of the audio: \
						     \'internal\' keyword for an internal tone generator; \
						     Name of a file to read audio from; \
						     \'loop_back\' keyword for using the recorded stream;')
parser.add_argument('-of','--output_filename',help='File to save audio to')
parser.add_argument('-fs','--sampling_rate',default=RATE,help='Sampling rate of the audio; default=16kHz. If playing from a file, this argument is ignored.')
parser.add_argument('-ch','--channels',default=CHANNELS,help='Number of channels ; default=1(mono). If playing from a file, this argument is ignored.')
parser.add_argument('--chunk',default=CHUNK,help='Chunk size used by pyaudio; default=256')
parser.add_argument('--siggen_freq',default=100,help='Signal generator frequency; default=100')

args = parser.parse_args()
RATE = int(args.sampling_rate)
CHANNELS = int(args.channels)
CHUNK = int(args.chunk)

def closeAndExit(pyAudioHandle, wavFileHandle=""):
    pyAudioHandle.terminate()
    if wavFileHandle != "":
        wavFileHandle.close()
    exit()


# DEVICENAME = 'ESP Audio'
# DEVICENAME = 'Built-in Output'


def getDevID(type, devName="", only_list=False):
    # type: input or output, optional device name
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')

    devID = -1

    if "input" == type:
        getStr = "maxInputChannels"
    elif "output" == type:
        getStr = "maxOutputChannels"
    else:
        print("Unknown type: \"", type, "\" passed to getDevID.")
        return devID

    devices = []
    for i in range(0, numdevices):
        if p.get_device_info_by_host_api_device_index(0, i).get(getStr) >= CHANNELS:
            name = p.get_device_info_by_host_api_device_index(0, i).get('name')
            devices.append([i, name])
            print(type," device id ", i, " - ", name)

    if len(devices) == 0:
        print(" There is no {} device that supports {} channels".format(type, CHANNELS))
        return devID, devName

    if only_list:
        return devID, devName

    if devName != "":
        for i in range(0, len(devices)):
            if devices[i][1] == devName:
                devID = devices[i][0]
    else:
        for i in range(0, len(devices)):
            print("\t", type, " device id ", devices[i][0], " - ", devices[i][1])
        response = input("-- Pick id of a recording devices from the list --\n>> ")
        if response.isdigit():
            input_id = int(response)
            for i in range(0, len(devices)):
                if devices[i][0] == input_id:
                    devID = devices[i][0]
                    devName = devices[i][1]
            if devID < 0:
                print("No device with that ID.")
                return devID, devName
        else:
            print("Bad device id..")
            return devID, devName

    if devID < 0:
        print("Could not find an ", type, " device with name matching \'", devName, "\'")

    return devID, devName


def decode(in_data, channels):
    """
    Convert a byte stream into a 2D numpy array with shape (chunk_size, channels)

    Samples are interleaved, so for a stereo stream with left channel  of [L0, L1, L2, ...] 
    and right channel of [R0, R1, R2, ...], the output is ordered as [L0, R0, L1, R1, ...]
    """
    # TODO: handle data type as parameter, convert between pyaudio/numpy types
    result = np.frombuffer(in_data, dtype='int16').astype('float32')

    chunk_length = len(result) / channels

    # Verify that chunk_length is a whole number
    assert chunk_length == int(chunk_length), 'function decode: chunk_length is fractional'

    result = np.reshape(result, (CHUNK, channels))
    #print(result.shape)
    return result


def encode(signal):
    """
    Convert a 2D numpy array into a byte stream for PyAudio

    Signal should be a numpy array with shape (chunk_size, channels)
    """

    assert signal.ndim == 2, 'function encode: input data is not 2D numpy array'
    assert signal.shape[1] == CHANNELS, 'function encode: input must have CHANNELS columns'

    interleaved = signal.flatten()

    # TODO: handle data type as parameter, convert between pyaudio/numpy types
    out_data = interleaved.astype('int16').tobytes()
    return out_data


sinusoid = Signal(amplitude=2 ** 12, frequency=int(args.siggen_freq), sampling_rate=RATE)

# In callback mode, PyAudio calla a user-defined callback function (1) whenever it needs new audio data to play 
# and/or when new recorded audio data becomes available. PyAudio calls the callback function in a separate thread.
# The callback function must have the fixed defined signature It must return a tuple containing frame_count frames 
# of audio data to output (for output streams) and a flag (pyaudio.paContinue, pyaudio.paComplete or pyaudio.paAborti).
# For input-only streams, the audio data portion of the return value is ignored.
#
# The audio stream calls the callback function repeatedly upon opening of the stream until that function returns 
# pyaudio.paComplete or pyaudio.paAbort, or until either pyaudio.PyAudio.Stream.stop or pyaudio.PyAudio.Stream.close is called.
# If the callback returns fewer frames than the frame_count argument, the stream automatically closes after those frames are played.

def callback(in_data, frame_count, time_info, flag):
    # using Numpy to convert to array for processing
    # audio_data = np.fromstring(in_data, dtype=np.float32)
    global wf, of, LOOPBACK, INTERNAL, SOURCE
    if wf and len(in_data)>0 :
        wf.writeframes(in_data)

    match SOURCE:
        case sources.LOOPBACK:
            return in_data, pyaudio.paContinue
        case sources.INTERNAL:
            s = sinusoid.sine(CHUNK).reshape(CHUNK,1)
            if CHANNELS == 1:
                out_data = s
            elif CHANNELS == 2:
                out_data = np.hstack((s, -s))
            return encode(out_data), pyaudio.paContinue
        case _:
            if of:
                return of.readframes(CHUNK), pyaudio.paContinue

    return in_data, pyaudio.paContinue

LENGTH = 512          # length of the filter
mu = 0.2              # rate of adjustment
w = np.array(np.zeros(LENGTH),dtype = 'float32')  # weights
m = np.array(np.zeros(LENGTH-1),dtype = 'float32')  # delay line for the samples
n = 0
def filter(ref_input, error):
    ''' 
    Uses an adaptive filter of LENGTH taps to filter ref_input, whose weights or coefficients are
    adjusted every sample period based on the error value.
    Even though this filter works on a CHUNK number of samples at a time, the processing i.e., filtering
    and adjusting the weights are done on a sample by sample basis.
    The filtered output is returned.
    '''
    global n
    global m, w, mu, LENGTH
    chunk_length = len(ref_input)
    assert chunk_length == len(error), 'function filter: ref_input and error must have the same number of elements'
    out = np.array([])
    tmp = np.append(m,ref_input)
    #print("m: {}, ref_input: {} tmp: {}".format(len(m),chunk_length, len(tmp)))
    for i in range(len(ref_input)):
        x = tmp[i:i+LENGTH]
        #print('x:{}'.format(x))
        s = np.dot(w, x)
        #print('s:{}'.format(s))
        w -= mu * error[i] * x 
        #print('w:{}'.format(w))
        out = np.append(out,int(s))

    #shift the ref_input into the delay line
    m = tmp[chunk_length:chunk_length+LENGTH-1]
    return out


def cancel_noise(in_data, frame_count, time_info, flag):

    global wf1, wf2

    in_data_1 = decode(in_data, CHANNELS)
    num_samples = len(in_data_1[:, 0])
    ref = in_data_1[:,0]
    err = in_data_1[:,1]
    l_data = sinusoid.sine(num_samples).reshape(num_samples,1)
    r_data = np.reshape(filter(ref, err),(CHUNK,1)).astype('int16')
    #print("{} {}".format(l_data.shape, r_data.shape))
    out_data = encode(np.hstack((l_data, r_data)))

    if wf1 and len(in_data)>0 :
        wf1.writeframes(in_data)
    if wf2 and len(out_data)>0 :
        wf2.writeframes(out_data)

    return out_data, pyaudio.paContinue
    #return in_data, pyaudio.paContinue


def quit(signo, _frame):
    print("Interrupted by %d, shutting down" % signo)
    waitThread.set()


##--- program start ---

p = pyaudio.PyAudio()

if args.list_devices:
    getDevID("input",devName="",only_list=True)
    getDevID("output",devName="",only_list=True)
    exit()

of = 0
SOURCE = sources.NULL
if(args.input_filename == 'loopback'):
	SOURCE = sources.LOOPBACK
elif (args.input_filename == 'internal'):
	SOURCE = sources.INTERNAL
elif (args.input_filename != None):
    SOURCE = sources.FILE
    of = wave.open(args.input_filename,'rb')
    nchnl,sampwidth,framerate,nframes, comptype, compname = of.getparams()
    RATE = framerate
    CHANNELS = nchnl

DEST = dest.NULL
if args.output_filename != None :
    DEST = dest.FILE

wf = 0
if DEST == dest.FILE :
    wf = wave.open(args.output_filename,'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)

wf1 = wave.open("mics.wav",'wb')
wf1.setnchannels(CHANNELS)
wf1.setsampwidth(p.get_sample_size(FORMAT))
wf1.setframerate(RATE)
wf2 = wave.open("speakers.wav",'wb')
wf2.setnchannels(CHANNELS)
wf2.setsampwidth(p.get_sample_size(FORMAT))
wf2.setframerate(RATE)
wf3 = wave.open("weights.wav",'wb')
wf3.setnchannels(LENGTH)
wf3.setsampwidth(p.get_sample_size(FORMAT))
wf3.setframerate(RATE)


device_idx=0
if SOURCE == sources.LOOPBACK or DEST == dest.FILE:
    input_dev, input_devname = getDevID("input",devName=args.devices[device_idx])
    if input_dev < 0:
        print("Something did not work out as expected. Exiting..")
        closeAndExit(p)
    device_idx = device_idx+1

if SOURCE != sources.NULL:
    if SOURCE == sources.LOOPBACK and len(args.devices) < 2 :
        print("Only one device provided, when both recording and playback devices are required. Exiting...")
        closeAndExit(p)
    output_dev, output_devname = getDevID("output",devName=args.devices[device_idx])
    if output_dev < 0:
        print("Something did not work out as expected. Exiting..")
        closeAndExit(p)

if SOURCE == sources.NULL and DEST == dest.NULL:
    print("Neither source nor destination is specified. Exiting...")
    closeAndExit(p)

if SOURCE != sources.NULL and DEST != dest.NULL and len(args.devices) < 2:
    print("Both recording and playback devices are needed. Exiting...")
    closeAndExit(p)

if SOURCE == sources.NULL and DEST == dest.FILE:
    stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                input_device_index=input_dev,
                frames_per_buffer=CHUNK,
                stream_callback=callback)
elif SOURCE != sources.LOOPBACK and DEST == dest.NULL:
    stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                output=True,
                output_device_index=output_dev,
                frames_per_buffer=CHUNK,
                stream_callback=callback)
elif SOURCE == sources.LOOPBACK or (DEST == dest.FILE and (SOURCE == sources.FILE or SOURCE == sources.INTERNAL)):
    stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                input_device_index=input_dev,
                output=True,
                output_device_index=output_dev,
                frames_per_buffer=CHUNK,
                stream_callback=callback)
else:
    print("...Exiting...")
    closeAndExit(p)


print("Audio will be recorded in this format:")
print("	", CHANNELS, " channels")
print("	", p.get_sample_size(FORMAT), " bytes (signed) per sample format")
print("	at ", RATE, "Hz sampling rate")


if DEST != dest.NULL :
    print("Recording from ", input_devname, " to ", args.output_filename )
if SOURCE != sources.NULL :
    print("Playing back on ", output_devname, "from ", end=" ")
if SOURCE == sources.FILE:
    print( args.input_filename)
elif SOURCE == sources.INTERNAL:
    print( "internal signal generator")
elif SOURCE == sources.LOOPBACK:
    print("recording device")

print("\nIf this does not match what you expect, modify the script and/or change options and start over.\n")
try:
    response = input("Press Ctrl+C to exit or Enter to continue..")
    # if(not response):
    pass
except KeyboardInterrupt:
    p.terminate()
    print()
    exit()

for sig in ('TERM', 'INT'):
    signal.signal(getattr(signal, 'SIG' + sig), quit)

print("... Press Ctrl+C to terminate")

stream.start_stream()

while not waitThread.is_set():
    # do_my_thing()
    waitThread.wait(60)

stream.close()
p.terminate()

if wf != 0:
    wf.close()
if wf1 != 0:
    wf1.close()
if wf2 != 0:
    wf2.close()
