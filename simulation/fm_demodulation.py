#!/usr/bin/env python3
"""Read simulated WBFM IQ, noncoherently demodulate it, and plot key stages."""

from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".mplconfig"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from fm_iq_io import read_iq_i16


OUT_DIR = Path(__file__).resolve().parent
FIG_DIR = OUT_DIR / "figures"

IQ_SAMPLE_RATE_HZ = 1_200_000.0
AUDIO_SAMPLE_RATE_HZ = 48_000.0
LOW_IF_HZ = 250_000.0
FM_DEVIATION_HZ = 75_000.0
PILOT_HZ = 19_000.0
STEREO_SUBCARRIER_HZ = 38_000.0


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


def bandpass_fir(low_hz: float, high_hz: float, sample_rate_hz: float, taps: int) -> np.ndarray:
    return lowpass_fir(high_hz, sample_rate_hz, taps) - lowpass_fir(low_hz, sample_rate_hz, taps)


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


def noncoherent_fm_demodulate(iq: np.ndarray) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    # 1. 求微分：低中频 FM 的瞬时角频率进入复微分信号幅度。
    differentiated = np.diff(iq, prepend=iq[0]) * IQ_SAMPLE_RATE_HZ

    # 2. 全波整流：取微分后复包络幅度，等价于对实信号微分结果做全波整流。
    rectified = np.abs(differentiated)

    # 3. 低通滤波：保留 0-100 kHz FM 立体声复合基带。
    h_composite = lowpass_fir(100_000.0, IQ_SAMPLE_RATE_HZ, 241)
    lowpassed = np.convolve(rectified, h_composite, mode="same")

    # 4. 去直流：去除低中频中心项 2*pi*f_if，留下与调制信号成比例的交流分量。
    dc_removed = lowpassed - np.mean(lowpassed)
    composite = dc_removed / max(float(np.max(np.abs(dc_removed))), 1e-12)
    stages = {
        "differentiated": differentiated,
        "rectified": rectified,
        "lowpassed": lowpassed,
        "dc_removed": composite,
    }
    return composite, stages


def decode_stereo(composite: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    decim1 = 5
    comp_240k = decimate_after_lpf(composite, 100_000.0, IQ_SAMPLE_RATE_HZ, decim1, 241)
    fs_comp = IQ_SAMPLE_RATE_HZ / decim1
    t = np.arange(comp_240k.size, dtype=np.float64) / fs_comp

    mono = np.convolve(comp_240k, lowpass_fir(15_000.0, fs_comp, 241), mode="same")
    stereo_band = np.convolve(comp_240k, bandpass_fir(23_000.0, 53_000.0, fs_comp, 241), mode="same")
    diff_mixed = 2.0 * stereo_band * np.cos(2.0 * np.pi * STEREO_SUBCARRIER_HZ * t)
    diff = np.convolve(diff_mixed, lowpass_fir(15_000.0, fs_comp, 241), mode="same")

    left_240k = mono + diff
    right_240k = mono - diff
    left = decimate_after_lpf(left_240k, 15_000.0, fs_comp, 5, 161)
    right = decimate_after_lpf(right_240k, 15_000.0, fs_comp, 5, 161)
    audio = np.column_stack((left, right))
    audio /= max(float(np.max(np.abs(audio))), 1e-12)
    return audio[:, 0], audio[:, 1], comp_240k


def plot_demodulation(iq: np.ndarray, composite: np.ndarray, stages: dict[str, np.ndarray], left: np.ndarray, right: np.ndarray) -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    t = np.arange(iq.size, dtype=np.float64) / IQ_SAMPLE_RATE_HZ
    n = int(0.004 * IQ_SAMPLE_RATE_HZ)

    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(t[:n] * 1e3, np.real(stages["differentiated"][:n]), linewidth=0.8)
    axes[0].set_ylabel("d/dt real")
    axes[0].set_title("Noncoherent FM Demodulation Stages")
    axes[1].plot(t[:n] * 1e3, stages["rectified"][:n], linewidth=0.8, color="tab:orange")
    axes[1].set_ylabel("Rectified")
    axes[2].plot(t[:n] * 1e3, stages["lowpassed"][:n], linewidth=0.8, color="tab:green")
    axes[2].set_ylabel("LPF")
    axes[3].plot(t[:n] * 1e3, stages["dc_removed"][:n], linewidth=0.8, color="tab:red")
    axes[3].set_ylabel("No DC")
    axes[3].set_xlabel("Time (ms)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIG_DIR / "04_noncoherent_demod_stages.png", dpi=150)
    plt.close(fig)

    freqs, power = spectrum_db(composite, IQ_SAMPLE_RATE_HZ)
    fig, ax = plt.subplots(figsize=(12, 5))
    mask = (freqs >= 0.0) & (freqs <= 60_000.0)
    ax.plot(freqs[mask] / 1e3, power[mask])
    ax.axvline(19.0, color="tab:orange", linestyle="--", linewidth=1.0, label="19 kHz pilot")
    ax.axvspan(23.0, 53.0, color="tab:green", alpha=0.12, label="Recovered L-R band")
    ax.set_title("Recovered Stereo Composite Spectrum")
    ax.set_xlabel("Frequency (kHz)")
    ax.set_ylabel("Magnitude (dB)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "05_recovered_composite_spectrum.png", dpi=150)
    plt.close(fig)

    ta = np.arange(left.size, dtype=np.float64) / AUDIO_SAMPLE_RATE_HZ
    m = int(0.030 * AUDIO_SAMPLE_RATE_HZ)
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(ta[:m] * 1e3, left[:m], label="Recovered Left")
    ax.plot(ta[:m] * 1e3, right[:m], label="Recovered Right", alpha=0.8)
    ax.set_title("Recovered Stereo Audio")
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Amplitude")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / "06_recovered_stereo_audio.png", dpi=150)
    plt.close(fig)


def main() -> int:
    iq_path = OUT_DIR / "FM_Modulation.iq"
    iq = read_iq_i16(iq_path)
    composite, stages = noncoherent_fm_demodulate(iq)
    left, right, _ = decode_stereo(composite)
    plot_demodulation(iq, composite, stages, left, right)

    print(f"read={iq_path}")
    print(f"samples={iq.size}")
    print(f"iq_sample_rate_hz={IQ_SAMPLE_RATE_HZ:.0f}")
    print(f"low_if_hz={LOW_IF_HZ:.0f}")
    print(f"audio_sample_rate_hz={AUDIO_SAMPLE_RATE_HZ:.0f}")
    print(f"audio_samples={left.size}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
