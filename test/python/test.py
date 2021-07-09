#!/usr/bin/env python3

import numpy as np
from scipy.io import wavfile

sampleRate = 48000
freq_zero = 1200
freq_one = 2400

baud=1200
duration = 1/baud

t = np.linspace(0, duration, sampleRate * duration, endpoint=False)

one_bit  = np.sin(freq_one  * 2 * np.pi * t)
zero_bit = np.sin(freq_zero * 2 * np.pi * t)

y = np.concatenate([
    np.tile(one_bit, 100),
    np.tile(np.concatenate([zero_bit, one_bit]), 5),
    np.tile(one_bit, 100)
    ])

wavfile.write('test.wav', sampleRate, y)
