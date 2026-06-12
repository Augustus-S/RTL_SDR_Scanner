#!/usr/bin/env python3
"""
Analyze raw RTL-SDR unsigned 8-bit interleaved IQ files captured by
example/fm_broadcast_player --debug on.
"""

import argparse
import logging
from pathlib import Path

import numpy as np

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger("analyze_fm_iq")


def read_u8_iq(path: Path, max_samples: int | None) -> np.ndarray:
    count = -1 if max_samples is None else max_samples * 2
    raw = np.fromfile(path, dtype=np.uint8, count=count)
    if raw.size < 2:
        raise ValueError(f"{path} does not contain enough IQ bytes")
    if raw.size % 2:
        raw = raw[:-1]

    i = (raw[0::2].astype(np.float32) - 127.5) / 127.5
    q = (raw[1::2].astype(np.float32) - 127.5) / 127.5
    return i + 1j * q


def power_db(x: np.ndarray) -> float:
    p = np.mean(np.abs(x) ** 2)
    return 10.0 * np.log10(max(float(p), 1e-20))


def fm_discriminator(iq: np.ndarray) -> np.ndarray:
    if iq.size < 2:
        return np.empty(0, dtype=np.float32)
    z = iq[1:] * np.conj(iq[:-1])
    return np.angle(z).astype(np.float32)


def averaged_spectrum(iq: np.ndarray, sample_rate: float, fft_size: int) -> tuple[np.ndarray, np.ndarray]:
    if iq.size < fft_size:
        raise ValueError(f"Need at least {fft_size} IQ samples for FFT analysis")

    usable = (iq.size // fft_size) * fft_size
    blocks = iq[:usable].reshape(-1, fft_size)
    window = np.hanning(fft_size).astype(np.float32)
    spec = np.fft.fftshift(np.fft.fft(blocks * window, axis=1), axes=1)
    power = np.mean(np.abs(spec) ** 2, axis=0)
    power_dbfs = 10.0 * np.log10(np.maximum(power / (np.sum(window ** 2) * fft_size), 1e-20))
    freqs = np.fft.fftshift(np.fft.fftfreq(fft_size, d=1.0 / sample_rate))
    return freqs, power_dbfs


def print_top_peaks(freqs: np.ndarray, spectrum_db: np.ndarray, limit: int) -> None:
    if limit <= 0:
        return
    order = np.argsort(spectrum_db)[::-1]
    chosen: list[int] = []
    min_sep_bins = max(1, len(freqs) // 200)
    for idx in order:
        if all(abs(int(idx) - prev) >= min_sep_bins for prev in chosen):
            chosen.append(int(idx))
        if len(chosen) >= limit:
            break

    logger.info("Top spectrum peaks:")
    for idx in chosen:
        logger.info("  %+10.1f Hz  %8.2f dB", freqs[idx], spectrum_db[idx])


def band_power(freqs: np.ndarray, spectrum_db: np.ndarray, lo_hz: float, hi_hz: float) -> float:
    mask = (freqs >= lo_hz) & (freqs <= hi_hz)
    if not np.any(mask):
        return float("nan")
    lin = 10.0 ** (spectrum_db[mask] / 10.0)
    return 10.0 * np.log10(max(float(np.mean(lin)), 1e-20))


def maybe_plot(freqs: np.ndarray, spectrum_db: np.ndarray, output: Path | None) -> None:
    if output is None:
        return
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(14, 6))
    ax.plot(freqs / 1e3, spectrum_db, linewidth=0.8)
    ax.set_xlabel("Baseband frequency (kHz)")
    ax.set_ylabel("Power (dB)")
    ax.set_title("RTL-SDR IQ Spectrum")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output, dpi=140)
    logger.info("Saved spectrum plot to %s", output)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze RTL-SDR raw unsigned 8-bit IQ files.")
    parser.add_argument("iq_file", type=Path, help="Raw unsigned 8-bit interleaved IQ file")
    parser.add_argument("--sample-rate", type=float, default=1_200_000.0, help="IQ sample rate, default 1200000")
    parser.add_argument("--fft-size", type=int, default=16384, help="FFT size, default 16384")
    parser.add_argument("--max-samples", type=int, default=None, help="Limit samples read from the IQ file")
    parser.add_argument("--peaks", type=int, default=8, help="Number of spectrum peaks to print")
    parser.add_argument("--station-offset", type=float, default=-250_000.0,
                        help="Expected station offset in captured IQ, default -250000 for --offset on")
    parser.add_argument("--plot", type=Path, default=None, help="Optional PNG path for spectrum plot")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    iq = read_u8_iq(args.iq_file, args.max_samples)
    duration = iq.size / args.sample_rate

    logger.info("File: %s", args.iq_file)
    logger.info("IQ samples: %d, duration: %.3f s, sample_rate: %.0f S/s", iq.size, duration, args.sample_rate)
    logger.info("IQ RMS: %.6f, IQ power: %.2f dB", float(np.sqrt(np.mean(np.abs(iq) ** 2))), power_db(iq))
    logger.info("I mean: %.6f, Q mean: %.6f", float(np.mean(iq.real)), float(np.mean(iq.imag)))

    disc = fm_discriminator(iq)
    if disc.size:
        logger.info("FM discriminator angle RMS: %.6f rad, peak: %.6f rad",
                    float(np.sqrt(np.mean(disc ** 2))), float(np.max(np.abs(disc))))

    freqs, spectrum_db = averaged_spectrum(iq, args.sample_rate, args.fft_size)
    logger.info("Full-band median power: %.2f dB", float(np.median(spectrum_db)))
    logger.info("DC +/-5 kHz power: %.2f dB", band_power(freqs, spectrum_db, -5_000, 5_000))
    logger.info("Expected station band %.0f +/-100 kHz power: %.2f dB",
                args.station_offset, band_power(freqs, spectrum_db, args.station_offset - 100_000,
                                                args.station_offset + 100_000))
    opposite_offset = -args.station_offset
    logger.info("Opposite offset band %.0f +/-100 kHz power: %.2f dB",
                opposite_offset, band_power(freqs, spectrum_db, opposite_offset - 100_000,
                                            opposite_offset + 100_000))
    logger.info("Wide FM baseband -600..+600 kHz peak: %.2f dB @ %.1f Hz",
                float(np.max(spectrum_db)), float(freqs[int(np.argmax(spectrum_db))]))
    print_top_peaks(freqs, spectrum_db, args.peaks)
    maybe_plot(freqs, spectrum_db, args.plot)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
