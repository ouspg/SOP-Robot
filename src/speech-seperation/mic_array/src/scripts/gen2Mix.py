import soundfile as sf
from scipy.io import wavfile
import numpy as np
from math import ceil
import copy

data_type = ['train-clean-100', 'dev-clean', 'test-clean']

LIBRISPEECH_PATH = ""
TASKFILE_TR = '{0}/LibriSpeech/train-clean-100/task.txt'.format(LIBRISPEECH_PATH)
TASKFILE_CV = '{0}/LibriSpeech/dev/task.txt'.format(LIBRISPEECH_PATH)
TASKFILE_TT = '{0}/LibriSpeech/test/task.txt'.format(LIBRISPEECH_PATH)
MIXSCP = '{0}/{1}/mix.scp'.format(LIBRISPEECH_PATH)
SPK1SCP = '{0}/{1}/spk1.scp'.format(LIBRISPEECH_PATH)
SPK2SCP = '{0}/{1}/spk2.scp'.format(LIBRISPEECH_PATH)

INT16_FAC = (2**15)-1

def gen2MixFile(file1, file2, key, data_type):
    samples1, sample_rate1 = sf.read(file1)
    samples2, sample_rate2 = sf.read(file2)

    # Find length of longest signal
    maxlength = max(len(samples1),len(samples2))

    # Pad each signal to the length of the longest signal
    samples1 = np.pad(samples1, (0, maxlength - len(samples1)), 'constant', constant_values=(0))
    samples2 = np.pad(samples2, (0, maxlength - len(samples2)), 'constant', constant_values=(0))

    # Combine series together
    mixed_series = samples1 + samples2

    # Pad 3 wav files to whole number of seconds
    extrapadding = ceil(len(mixed_series) / sample_rate1) * sample_rate1 - len(mixed_series)
    mixed_series = np.pad(mixed_series, (0, extrapadding), 'constant', constant_values=(0))
    samples1 = np.pad(samples1, (0,extrapadding), 'constant', constant_values=(0))
    samples2 = np.pad(samples2, (0,extrapadding), 'constant', constant_values=(0))

    mix_filename = "{0}/{1}/mix/mix{2}.wav".format(LIBRISPEECH_PATH, data_type, key)
    spk1_filename = "{0}/{1}/spk1/spk1_{2}.wav".format(LIBRISPEECH_PATH, data_type, key)
    spk2_filename = "{0}/{1}/spk2/spk2_{2}.wav".format(LIBRISPEECH_PATH, data_type, key)

    wavwrite(np.asarray(mixed_series, dtype=np.float), sample_rate1, mix_filename)
    wavwrite(np.asarray(samples1, dtype=np.float), sample_rate1, spk1_filename)
    wavwrite(np.asarray(samples2, dtype=np.float), sample_rate1, spk2_filename)

    with open(MIXSCP.format(data_type), 'a') as mix_scp:
        mix_scp.write("{0} {1}\n".format(key, mix_filename))

    with open(SPK1SCP.format(data_type), 'a') as spk1_scp:
        spk1_scp.write("{0} {1}\n".format(key, spk1_filename))

    with open(SPK2SCP.format(data_type), 'a') as spk2_scp:
        spk2_scp.write("{0} {1}\n".format(key, spk2_filename))

def wavwrite(y, fs, filename):
    """
    Write a sound file from an array with the sound and the sampling rate
    y: floating point array of one dimension, fs: sampling rate
    filename: name of file to create
    """
                                
    x = copy.deepcopy(y)                         # copy array
    x *= INT16_FAC                               # scaling floating point -1 to 1 range signal to int16 range
    x = np.int16(x)                              # converting to int16 type
    wavfile.write(filename, fs, x)

if __name__ == '__main__':
    key = 0

    with open(TASKFILE_TR) as tr:
        tasks_tr = tr.readlines()

    with open(TASKFILE_CV) as cv:
        tasks_cv = cv.readlines()

    with open(TASKFILE_TT) as tt:
        tasks_tt = tt.readlines()

    for task in tasks_tr:
        spk1, spk2 = task.strip().split()
        gen2MixFile(spk1, spk2, key, data_type[0])
        key += 1

    for task in tasks_cv:
        spk1, spk2 = task.strip().split()
        gen2MixFile(spk1, spk2, key, data_type[1])
        key += 1

    for task in tasks_tt:
        spk1, spk2 = task.strip().split()
        gen2MixFile(spk1, spk2, key, data_type[2])
        key += 1