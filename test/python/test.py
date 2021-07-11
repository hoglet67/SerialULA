#!/usr/bin/env python3

import numpy as np
from scipy.io import wavfile

sampleRate = 48000

freq_start = 1200
freq_zero  = 1200
freq_one   = 6000
freq_stop  = 6000

t_start = np.linspace(0, 1/freq_start,     sampleRate / freq_start, endpoint=False)
t_one   = np.linspace(0, 2/freq_one,   2 * sampleRate / freq_one,   endpoint=False)
t_zero  = np.linspace(0, 1/freq_zero,      sampleRate / freq_zero,  endpoint=False)
t_stop  = np.linspace(0, 2/freq_stop,  2 * sampleRate / freq_stop,  endpoint=False)

start_bit = np.cos(freq_start  * 2 * np.pi * t_start)
one_bit   = np.cos(freq_one    * 2 * np.pi * t_one)
zero_bit  = np.cos(freq_zero   * 2 * np.pi * t_zero)
stop_bit  = np.cos(freq_stop   * 2 * np.pi * t_stop)

y = np.concatenate([
    np.tile(one_bit, 100),
    start_bit,
    np.tile(np.concatenate([zero_bit, one_bit]), 4),
    stop_bit,
    start_bit,
    np.tile(np.concatenate([zero_bit, one_bit]), 4),
    stop_bit,
    np.tile(one_bit, 100)
    ])

wavfile.write('test.wav', sampleRate, y)
