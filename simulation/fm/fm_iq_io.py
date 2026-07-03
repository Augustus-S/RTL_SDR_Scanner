#!/usr/bin/env python3
"""Read and write interleaved signed 16-bit IQ files."""

from __future__ import annotations

from pathlib import Path

import numpy as np


def write_iq_i16(path: Path, iq: np.ndarray, scale: float = 0.90) -> None:
    """Write complex IQ as little-endian int16 interleaved I,Q samples."""
    if iq.ndim != 1:
        raise ValueError("iq must be a one-dimensional complex array")
    if iq.size == 0:
        raise ValueError("iq is empty")

    peak = float(max(np.max(np.abs(iq.real)), np.max(np.abs(iq.imag)), 1e-12))
    gain = scale * 32767.0 / peak
    interleaved = np.empty(iq.size * 2, dtype="<i2")
    interleaved[0::2] = np.clip(np.rint(iq.real * gain), -32768, 32767).astype("<i2")
    interleaved[1::2] = np.clip(np.rint(iq.imag * gain), -32768, 32767).astype("<i2")
    interleaved.tofile(path)


def read_iq_i16(path: Path, max_samples: int | None = None) -> np.ndarray:
    """Read little-endian int16 interleaved I,Q samples as normalized complex64."""
    count = -1 if max_samples is None else max_samples * 2
    raw = np.fromfile(path, dtype="<i2", count=count)
    if raw.size < 2:
        raise ValueError(f"{path} does not contain enough IQ data")
    if raw.size % 2:
        raw = raw[:-1]

    i = raw[0::2].astype(np.float32) / 32768.0
    q = raw[1::2].astype(np.float32) / 32768.0
    return (i + 1j * q).astype(np.complex64)
