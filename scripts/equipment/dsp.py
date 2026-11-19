"""Analysis of captured samples."""

import json

import numpy as np


def read_sigmf(base):
    """Reads a limeTRX SigMF recording (<base>.sigmf-data / .sigmf-meta) of a single channel.
    Returns (complex64 samples normalized to full scale = 1.0, sample rate)."""
    with open(base + ".sigmf-meta") as f:
        meta = json.load(f)["global"]
    if meta.get("core:datatype") != "ci16_le":
        raise ValueError("unsupported SigMF datatype: %s" % meta.get("core:datatype"))
    if meta.get("core:num_channels", 1) != 1:
        raise ValueError("expected a single channel recording")
    raw = np.fromfile(base + ".sigmf-data", dtype="<i2").astype(np.float32)
    iq = (raw[0::2] + 1j * raw[1::2]).astype(np.complex64) / 32768.0
    return iq, float(meta["core:sample_rate"])


def find_peak(iq, sample_rate, fft_size=8192, skip=0):
    """Finds the strongest spectral component, excluding DC.

    Same method as limeOEM (src/OEMTesting.cpp), so the dBFS limits are interchangeable:
    a single rectangular-window FFT, power = |X|^2 / N^2, frequency of the peak bin.
    Returns (frequency offset in Hz, amplitude in dBFS).
    """
    block = iq[skip:skip + fft_size]
    if len(block) < fft_size:
        raise ValueError("not enough samples for the FFT: %d < %d" % (len(block), fft_size))
    spectrum = np.fft.fft(block)
    power = (spectrum.real ** 2 + spectrum.imag ** 2) / (fft_size * fft_size)
    with np.errstate(divide="ignore"):
        dbfs = np.where(power > 0, 10 * np.log10(power), -150.0)
    index = 1 + int(np.argmax(dbfs[1:]))  # skip DC bin
    frequency = index * sample_rate / fft_size
    if frequency > sample_rate / 2:
        frequency -= sample_rate
    return frequency, float(dbfs[index])
