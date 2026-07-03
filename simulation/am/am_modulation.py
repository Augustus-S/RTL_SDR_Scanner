#!/usr/bin/env python3
"""Simulate conventional AM modulation and save 16-bit interleaved IQ."""

from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".mplconfig"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from am_iq_io import write_iq_i16


OUT_DIR = Path(__file__).resolve().parent
FIG_DIR = OUT_DIR / "figures"

CARRIER_HZ = 1_000_000.0
IQ_SAMPLE_RATE_HZ = 240_000.0
AUDIO_SAMPLE_RATE_HZ = 48_000.0
LOW_IF_HZ = 50_000.0
MODULATION_INDEX = 0.70
DURATION_S = 0.50


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


def make_audio_program(t: np.ndarray) -> np.ndarray:
    audio = (
        0.65 * np.sin(2.0 * np.pi * 1_000.0 * t)
        + 0.25 * np.sin(2.0 * np.pi * 2_400.0 * t)
        + 0.12 * np.sin(2.0 * np.pi * 4_200.0 * t)
    )
    audio /= max(float(np.max(np.abs(audio))), 1e-12)
    return audio.astype(np.float64)


def am_modulate(audio: np.ndarray, sample_rate_hz: float) -> tuple[np.ndarray, np.ndarray]:
    # 常规 AM 需要保留载波，包络 1 + mu*m(t) 必须保持为正，避免过调制。
    envelope = 1.0 + MODULATION_INDEX * audio
    t = np.arange(audio.size, dtype=np.float64) / sample_rate_hz
    iq = envelope * np.exp(1j * 2.0 * np.pi * LOW_IF_HZ * t)
    return iq.astype(np.complex64), envelope


def plot_modulation(t: np.ndarray, audio: np.ndarray, envelope: np.ndarray, iq: np.ndarray) -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)

    n_audio = int(0.030 * IQ_SAMPLE_RATE_HZ)
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t[:n_audio] * 1e3, audio[:n_audio], label="Message")
    ax.plot(t[:n_audio] * 1e3, envelope[:n_audio], label="AM envelope", alpha=0.85)
    ax.plot(t[:n_audio] * 1e3, -envelope[:n_audio], color="tab:orange", alpha=0.45)
    ax.set_title("AM Audio Message and Envelope")
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Amplitude")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "01_am_message_envelope.png", dpi=150)
    plt.close(fig)

    n_if = int(0.004 * IQ_SAMPLE_RATE_HZ)
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t[:n_if] * 1e3, np.real(iq[:n_if]), linewidth=0.8, label="I / real low-IF")
    ax.plot(t[:n_if] * 1e3, envelope[:n_if], color="tab:orange", linewidth=1.0, label="Envelope")
    ax.plot(t[:n_if] * 1e3, -envelope[:n_if], color="tab:orange", linewidth=1.0, alpha=0.45)
    ax.set_title("Generated AM Low-IF Waveform")
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Amplitude")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "02_generated_am_waveform.png", dpi=150)
    plt.close(fig)

    freqs, power = spectrum_db(iq, IQ_SAMPLE_RATE_HZ)
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(freqs / 1e3, power)
    ax.axvline(LOW_IF_HZ / 1e3, color="tab:red", linestyle="--", linewidth=1.0, label="Carrier / low IF")
    ax.axvspan((LOW_IF_HZ - 5_000.0) / 1e3, (LOW_IF_HZ + 5_000.0) / 1e3, color="tab:green", alpha=0.12, label="AM sidebands")
    ax.set_title("Generated AM IQ Spectrum")
    ax.set_xlabel("Baseband frequency (kHz)")
    ax.set_ylabel("Magnitude (dB)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "03_generated_am_iq_spectrum.png", dpi=150)
    plt.close(fig)


def main() -> int:
    t = np.arange(int(DURATION_S * IQ_SAMPLE_RATE_HZ), dtype=np.float64) / IQ_SAMPLE_RATE_HZ
    audio = make_audio_program(t)
    iq, envelope = am_modulate(audio, IQ_SAMPLE_RATE_HZ)

    write_iq_i16(OUT_DIR / "AM_Modulation.iq", iq)
    plot_modulation(t, audio, envelope, iq)
    print(f"carrier_hz={CARRIER_HZ:.0f}")
    print(f"iq_sample_rate_hz={IQ_SAMPLE_RATE_HZ:.0f}")
    print(f"low_if_hz={LOW_IF_HZ:.0f}")
    print(f"modulation_index={MODULATION_INDEX:.2f}")
    print(f"samples={iq.size}")
    print(f"wrote={OUT_DIR / 'AM_Modulation.iq'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
