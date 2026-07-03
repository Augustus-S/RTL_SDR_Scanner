#!/usr/bin/env python3
"""Read simulated AM IQ, envelope-demodulate it, and plot key stages."""

from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".mplconfig"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from am_iq_io import read_iq_i16


OUT_DIR = Path(__file__).resolve().parent
FIG_DIR = OUT_DIR / "figures"

IQ_SAMPLE_RATE_HZ = 240_000.0
AUDIO_SAMPLE_RATE_HZ = 48_000.0
LOW_IF_HZ = 50_000.0
AUDIO_CUTOFF_HZ = 8_000.0


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


def decimate_after_lpf(x: np.ndarray, cutoff_hz: float, sample_rate_hz: float, factor: int, taps: int) -> np.ndarray:
    h = lowpass_fir(cutoff_hz, sample_rate_hz, taps)
    y = np.convolve(x, h, mode="same")
    return y[::factor]


def envelope_demodulate(iq: np.ndarray) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    # 1. 包络检波：常规 AM 的信息直接写在载波幅度上，复 IQ 取模即可得到包络。
    envelope = np.abs(iq)

    # 2. 去直流：包络中的常量 1 对应未调制载波幅度，音频只在交流分量里。
    dc_removed = envelope - np.mean(envelope)

    # 3. 低通滤波：只保留音频带宽，抑制量化噪声和包络估计纹波。
    audio_wide = np.convolve(dc_removed, lowpass_fir(AUDIO_CUTOFF_HZ, IQ_SAMPLE_RATE_HZ, 241), mode="same")

    # 4. 降采样：240 kS/s -> 48 kS/s，得到常见音频采样率。
    audio = decimate_after_lpf(audio_wide, AUDIO_CUTOFF_HZ, IQ_SAMPLE_RATE_HZ, 5, 161)
    audio /= max(float(np.max(np.abs(audio))), 1e-12)
    stages = {
        "envelope": envelope,
        "dc_removed": dc_removed,
        "audio_wide": audio_wide,
        "audio": audio,
    }
    return audio, stages


def plot_demodulation(iq: np.ndarray, stages: dict[str, np.ndarray]) -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    t = np.arange(iq.size, dtype=np.float64) / IQ_SAMPLE_RATE_HZ
    n = int(0.030 * IQ_SAMPLE_RATE_HZ)

    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=False)
    axes[0].plot(t[:n] * 1e3, np.real(iq[:n]), linewidth=0.7)
    axes[0].set_title("AM Envelope Demodulation Stages")
    axes[0].set_ylabel("Low-IF real")
    axes[1].plot(t[:n] * 1e3, stages["envelope"][:n], linewidth=0.8, color="tab:orange")
    axes[1].set_ylabel("Envelope")
    axes[2].plot(t[:n] * 1e3, stages["dc_removed"][:n], linewidth=0.8, color="tab:green")
    axes[2].set_ylabel("No DC")
    axes[3].plot(t[:n] * 1e3, stages["audio_wide"][:n], linewidth=0.8, color="tab:red")
    axes[3].set_ylabel("LPF audio")
    axes[3].set_xlabel("Time (ms)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIG_DIR / "04_am_demodulation_stages.png", dpi=150)
    plt.close(fig)

    freqs, power = spectrum_db(stages["dc_removed"], IQ_SAMPLE_RATE_HZ)
    fig, ax = plt.subplots(figsize=(12, 5))
    mask = (freqs >= 0.0) & (freqs <= 12_000.0)
    ax.plot(freqs[mask] / 1e3, power[mask])
    ax.axvline(1.0, color="tab:red", linestyle="--", linewidth=1.0, label="1 kHz")
    ax.axvline(2.4, color="tab:orange", linestyle="--", linewidth=1.0, label="2.4 kHz")
    ax.axvline(4.2, color="tab:green", linestyle="--", linewidth=1.0, label="4.2 kHz")
    ax.set_title("Recovered AM Audio Spectrum Before Decimation")
    ax.set_xlabel("Frequency (kHz)")
    ax.set_ylabel("Magnitude (dB)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "05_recovered_am_audio_spectrum.png", dpi=150)
    plt.close(fig)

    audio = stages["audio"]
    ta = np.arange(audio.size, dtype=np.float64) / AUDIO_SAMPLE_RATE_HZ
    m = int(0.050 * AUDIO_SAMPLE_RATE_HZ)
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(ta[:m] * 1e3, audio[:m], label="Recovered audio")
    ax.set_title("Recovered AM Audio")
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Amplitude")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "06_recovered_am_audio.png", dpi=150)
    plt.close(fig)


def main() -> int:
    iq_path = OUT_DIR / "AM_Modulation.iq"
    iq = read_iq_i16(iq_path)
    audio, stages = envelope_demodulate(iq)
    plot_demodulation(iq, stages)

    print(f"read={iq_path}")
    print(f"samples={iq.size}")
    print(f"iq_sample_rate_hz={IQ_SAMPLE_RATE_HZ:.0f}")
    print(f"low_if_hz={LOW_IF_HZ:.0f}")
    print(f"audio_sample_rate_hz={AUDIO_SAMPLE_RATE_HZ:.0f}")
    print(f"audio_samples={audio.size}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
