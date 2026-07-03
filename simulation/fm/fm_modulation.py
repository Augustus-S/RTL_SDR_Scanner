#!/usr/bin/env python3
"""Simulate stereo WBFM modulation and save 16-bit interleaved IQ."""

from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".mplconfig"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from fm_iq_io import write_iq_i16


OUT_DIR = Path(__file__).resolve().parent
FIG_DIR = OUT_DIR / "figures"

CARRIER_HZ = 88_000_000.0
IQ_SAMPLE_RATE_HZ = 1_200_000.0
AUDIO_SAMPLE_RATE_HZ = 48_000.0
LOW_IF_HZ = 250_000.0
FM_DEVIATION_HZ = 75_000.0
PILOT_HZ = 19_000.0
STEREO_SUBCARRIER_HZ = 38_000.0
DURATION_S = 0.25


def lowpass_fir(cutoff_hz: float, sample_rate_hz: float, taps: int) -> np.ndarray:
    if taps % 2 == 0:
        raise ValueError("FIR tap count must be odd")
    n = np.arange(taps, dtype=np.float64)
    m = n - (taps - 1) / 2.0
    fc = cutoff_hz / sample_rate_hz
    h = 2.0 * fc * np.sinc(2.0 * fc * m)
    h *= np.hamming(taps)
    h /= np.sum(h)
    return h


def spectrum_db(x: np.ndarray, sample_rate_hz: float) -> tuple[np.ndarray, np.ndarray]:
    n = min(65536, 1 << int(np.floor(np.log2(x.size))))
    window = np.hanning(n)
    spec = np.fft.fftshift(np.fft.fft(x[:n] * window))
    power = 20.0 * np.log10(np.maximum(np.abs(spec) / np.sum(window), 1e-12))
    freqs = np.fft.fftshift(np.fft.fftfreq(n, d=1.0 / sample_rate_hz))
    return freqs, power


def make_stereo_program(t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    left = (
        0.55 * np.sin(2.0 * np.pi * 1_000.0 * t)
        + 0.25 * np.sin(2.0 * np.pi * 3_000.0 * t)
    )
    right = (
        0.50 * np.sin(2.0 * np.pi * 1_500.0 * t)
        + 0.22 * np.sin(2.0 * np.pi * 4_000.0 * t)
    )
    return left.astype(np.float64), right.astype(np.float64)


def make_stereo_composite(t: np.ndarray, left: np.ndarray, right: np.ndarray) -> np.ndarray:
    mono = 0.5 * (left + right)
    diff = 0.5 * (left - right)
    pilot = 0.10 * np.sin(2.0 * np.pi * PILOT_HZ * t)
    dsb_sc = diff * np.cos(2.0 * np.pi * STEREO_SUBCARRIER_HZ * t)
    composite = 0.90 * mono + pilot + 0.90 * dsb_sc
    composite /= max(float(np.max(np.abs(composite))), 1e-12)
    return composite


def wbfm_modulate(composite: np.ndarray, sample_rate_hz: float) -> np.ndarray:
    phase_deviation = 2.0 * np.pi * FM_DEVIATION_HZ * np.cumsum(composite) / sample_rate_hz
    t = np.arange(composite.size, dtype=np.float64) / sample_rate_hz
    phase = 2.0 * np.pi * LOW_IF_HZ * t + phase_deviation
    return np.exp(1j * phase).astype(np.complex64)


def plot_modulation(t: np.ndarray, left: np.ndarray, right: np.ndarray, composite: np.ndarray, iq: np.ndarray) -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)

    audio_samples = int(0.010 * IQ_SAMPLE_RATE_HZ)
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t[:audio_samples] * 1e3, left[:audio_samples], label="Left")
    ax.plot(t[:audio_samples] * 1e3, right[:audio_samples], label="Right", alpha=0.8)
    ax.set_title("Stereo Program Source")
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Amplitude")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "01_stereo_audio_source.png", dpi=150)
    plt.close(fig)

    freqs, power = spectrum_db(composite, IQ_SAMPLE_RATE_HZ)
    fig, ax = plt.subplots(figsize=(12, 5))
    mask = (freqs >= 0.0) & (freqs <= 60_000.0)
    ax.plot(freqs[mask] / 1e3, power[mask])
    ax.axvline(19.0, color="tab:orange", linestyle="--", linewidth=1.0, label="19 kHz pilot")
    ax.axvspan(23.0, 53.0, color="tab:green", alpha=0.12, label="L-R DSB-SC")
    ax.set_title("FM Stereo Composite Multiplex Spectrum")
    ax.set_xlabel("Frequency (kHz)")
    ax.set_ylabel("Magnitude (dB)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "02_stereo_composite_spectrum.png", dpi=150)
    plt.close(fig)

    freqs, power = spectrum_db(iq, IQ_SAMPLE_RATE_HZ)
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(freqs / 1e3, power)
    ax.axvline(LOW_IF_HZ / 1e3, color="tab:red", linestyle="--", linewidth=1.0, label="250 kHz low IF")
    ax.set_title("Generated WBFM IQ Spectrum")
    ax.set_xlabel("Baseband frequency (kHz)")
    ax.set_ylabel("Magnitude (dB)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "03_generated_iq_spectrum.png", dpi=150)
    plt.close(fig)


def main() -> int:
    t = np.arange(int(DURATION_S * IQ_SAMPLE_RATE_HZ), dtype=np.float64) / IQ_SAMPLE_RATE_HZ
    left, right = make_stereo_program(t)
    composite = make_stereo_composite(t, left, right)
    iq = wbfm_modulate(composite, IQ_SAMPLE_RATE_HZ)

    write_iq_i16(OUT_DIR / "FM_Modulation.iq", iq)
    plot_modulation(t, left, right, composite, iq)
    print(f"carrier_hz={CARRIER_HZ:.0f}")
    print(f"iq_sample_rate_hz={IQ_SAMPLE_RATE_HZ:.0f}")
    print(f"low_if_hz={LOW_IF_HZ:.0f}")
    print(f"fm_deviation_hz={FM_DEVIATION_HZ:.0f}")
    print(f"samples={iq.size}")
    print(f"wrote={OUT_DIR / 'FM_Modulation.iq'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
