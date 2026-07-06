"""Read and write RTL-SDR style unsigned 8-bit interleaved IQ files."""

from __future__ import annotations

from pathlib import Path

import numpy as np


def write_iq_u8(path: Path, iq: np.ndarray) -> None:
    """Write complex baseband samples as unsigned 8-bit interleaved I/Q."""
    path.parent.mkdir(parents=True, exist_ok=True)
    clipped_i = np.clip(np.real(iq), -127.0, 127.0)
    clipped_q = np.clip(np.imag(iq), -127.0, 127.0)

    interleaved = np.empty(iq.size * 2, dtype=np.uint8)
    interleaved[0::2] = np.clip(np.rint(clipped_i + 127.0), 0, 255).astype(np.uint8)
    interleaved[1::2] = np.clip(np.rint(clipped_q + 127.0), 0, 255).astype(np.uint8)
    interleaved.tofile(path)


def read_iq_u8(path: Path) -> np.ndarray:
    """Read unsigned 8-bit interleaved I/Q as centered complex samples."""
    raw = np.fromfile(path, dtype=np.uint8)
    if raw.size % 2 != 0:
        raise ValueError(f"IQ file has odd byte count: {path}")

    centered = raw.astype(np.float32) - 127.0
    return centered[0::2] + 1j * centered[1::2]
