"""Generate a synthetic ADS-B 1090ES PPM IQ capture.

The output is the complex baseband signal an RTL-SDR would deliver after being
tuned to the real 1090 MHz ADS-B carrier. It keeps the production decoder's
2 MS/s unsigned 8-bit interleaved IQ format.
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import sys
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".mplconfig"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

_IQ_IO_PATH = Path(__file__).resolve().parent / "ADS-B_iq_io.py"
_IQ_IO_SPEC = importlib.util.spec_from_file_location("mode_s_iq_io", _IQ_IO_PATH)
if _IQ_IO_SPEC is None or _IQ_IO_SPEC.loader is None:
    raise ImportError(f"Cannot load IQ helper: {_IQ_IO_PATH}")
_IQ_IO = importlib.util.module_from_spec(_IQ_IO_SPEC)
sys.modules[_IQ_IO_SPEC.name] = _IQ_IO
_IQ_IO_SPEC.loader.exec_module(_IQ_IO)
write_iq_u8 = _IQ_IO.write_iq_u8


CENTER_FREQ_HZ = 1_090_000_000
SAMPLE_RATE_HZ = 2_000_000
BITS_PER_SECOND = 1_000_000
SAMPLES_PER_US = SAMPLE_RATE_HZ // 1_000_000
SAMPLES_PER_BIT = SAMPLE_RATE_HZ // BITS_PER_SECOND
PREAMBLE_US = 8
PREAMBLE_SAMPLES = PREAMBLE_US * SAMPLES_PER_US
LONG_MSG_BITS = 112
MESSAGE_SAMPLES = PREAMBLE_SAMPLES + LONG_MSG_BITS * SAMPLES_PER_BIT
PREAMBLE_PULSES = (0, 2, 7, 9)
OUT_DIR = Path(__file__).resolve().parent

MODES_CHECKSUM_TABLE = [
    0x3935EA, 0x1C9AF5, 0xF1B77E, 0x78DBBF, 0xC397DB, 0x9E31E9, 0xB0E2F0, 0x587178,
    0x2C38BC, 0x161C5E, 0x0B0E2F, 0xFA7D13, 0x82C48D, 0xBE9842, 0x5F4C21, 0xD05C14,
    0x682E0A, 0x341705, 0xE5F186, 0x72F8C3, 0xC68665, 0x9CB936, 0x4E5C9B, 0xD8D449,
    0x939020, 0x49C810, 0x24E408, 0x127204, 0x093902, 0x049C81, 0xFDB444, 0x7EDA22,
    0x3F6D11, 0xE04C8C, 0x702646, 0x381323, 0xE3F395, 0x8E03CE, 0x4701E7, 0xDC7AF7,
    0x91C77F, 0xB719BB, 0xA476D9, 0xADC168, 0x56E0B4, 0x2B705A, 0x15B82D, 0xF52612,
    0x7A9309, 0xC2B380, 0x6159C0, 0x30ACE0, 0x185670, 0x0C2B38, 0x06159C, 0x030ACE,
    0x018567, 0xFF38B7, 0x80665F, 0xBFC92B, 0xA01E91, 0xAFF54C, 0x57FAA6, 0x2BFD53,
    0xEA04AD, 0x8AF852, 0x457C29, 0xDD4410, 0x6EA208, 0x375104, 0x1BA882, 0x0DD441,
    0xF91024, 0x7C8812, 0x3E4409, 0xE0D800, 0x706C00, 0x383600, 0x1C1B00, 0x0E0D80,
    0x0706C0, 0x038360, 0x01C1B0, 0x00E0D8, 0x00706C, 0x003836, 0x001C1B, 0xFFF409,
]

MODE_S_CHARSET = "?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????"


def bytes_to_bits(data: bytes, bit_count: int | None = None) -> list[int]:
    bits: list[int] = []
    for value in data:
        bits.extend((value >> shift) & 1 for shift in range(7, -1, -1))
    return bits if bit_count is None else bits[:bit_count]


def pack_bits(bits: list[int]) -> bytes:
    if len(bits) % 8 != 0:
        raise ValueError("bit count must be a multiple of 8")
    out = bytearray(len(bits) // 8)
    for index, bit in enumerate(bits):
        out[index // 8] |= (bit & 1) << (7 - index % 8)
    return bytes(out)


def mode_s_crc(data: bytes, bit_count: int) -> int:
    """Compute the 24-bit Mode-S parity over message bits before parity."""
    bits = bytes_to_bits(data, bit_count)
    crc = 0
    for index, bit in enumerate(bits):
        if bit:
            crc ^= MODES_CHECKSUM_TABLE[index]
    return crc & 0x00FFFFFF


def encode_callsign_me(callsign: str, type_code: int = 4, category: int = 0) -> bytes:
    """Build the 56-bit ADS-B ME field for an aircraft identification frame."""
    normalized = callsign.upper().ljust(8)[:8]
    bits = [(type_code >> shift) & 1 for shift in range(4, -1, -1)]
    bits.extend((category >> shift) & 1 for shift in range(2, -1, -1))
    for char in normalized:
        code = MODE_S_CHARSET.find(char)
        if code < 0:
            code = MODE_S_CHARSET.find(" ")
        bits.extend((code >> shift) & 1 for shift in range(5, -1, -1))
    return pack_bits(bits)


def make_df17_callsign_message(icao: int, callsign: str) -> bytes:
    """Create a valid 112-bit DF17 ADS-B extended squitter message."""
    if not 0 <= icao <= 0xFFFFFF:
        raise ValueError("ICAO address must fit in 24 bits")
    first_88 = bytearray()
    first_88.append((17 << 3) | 5)
    first_88.extend(icao.to_bytes(3, "big"))
    first_88.extend(encode_callsign_me(callsign))
    parity = mode_s_crc(bytes(first_88), 88)
    first_88.extend(parity.to_bytes(3, "big"))
    return bytes(first_88)


def render_ppm_message(bits: list[int], amplitude: float) -> np.ndarray:
    """Render Mode-S preamble and 112 PPM bits into baseband pulse samples."""
    envelope = np.zeros(MESSAGE_SAMPLES, dtype=np.float32)
    for sample_index in PREAMBLE_PULSES:
        envelope[sample_index] = amplitude

    data_offset = PREAMBLE_SAMPLES
    for bit_index, bit in enumerate(bits):
        sample_index = data_offset + bit_index * SAMPLES_PER_BIT + (0 if bit else 1)
        envelope[sample_index] = amplitude
    return envelope


def synthesize_capture(
    messages: list[bytes],
    amplitude: float,
    noise_std: float,
    gap_us: int,
    seed: int,
) -> np.ndarray:
    """Create a complete complex baseband capture containing one or more frames."""
    rng = np.random.default_rng(seed)
    gap_samples = gap_us * SAMPLES_PER_US
    total_samples = gap_samples + len(messages) * (MESSAGE_SAMPLES + gap_samples)
    iq = (rng.normal(0, noise_std, total_samples) + 1j * rng.normal(0, noise_std, total_samples)).astype(np.complex64)

    offset = gap_samples
    for message in messages:
        envelope = render_ppm_message(bytes_to_bits(message, LONG_MSG_BITS), amplitude)
        iq[offset : offset + MESSAGE_SAMPLES] += envelope.astype(np.complex64)
        offset += MESSAGE_SAMPLES + gap_samples
    return iq


def plot_capture(iq: np.ndarray, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    magnitude = np.abs(iq)
    time_us = np.arange(iq.size) / SAMPLES_PER_US

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(time_us, magnitude, linewidth=0.9)
    ax.set_title("Synthetic ADS-B 1090ES baseband magnitude")
    ax.set_xlabel("Time (us)")
    ax.set_ylabel("Magnitude")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(output_dir / "01_ADS-B_capture_magnitude.png", dpi=150)
    plt.close(fig)

    first_frame_start = int(np.argmax(magnitude > max(8.0, magnitude.max() * 0.5)))
    window = slice(max(0, first_frame_start - 2), first_frame_start + PREAMBLE_SAMPLES + 16)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.stem(np.arange(window.start, window.stop) / SAMPLES_PER_US, magnitude[window], basefmt=" ")
    ax.set_title("ADS-B preamble and first PPM bits")
    ax.set_xlabel("Time (us)")
    ax.set_ylabel("Magnitude")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(output_dir / "02_ADS-B_preamble_ppm.png", dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate synthetic ADS-B 1090ES IQ data.")
    parser.add_argument("--output", type=Path, default=OUT_DIR / "ADS-B_1090ES.iq")
    parser.add_argument("--icao", type=lambda value: int(value, 16), default=0xABCDEF)
    parser.add_argument("--callsign", default="RTLSDR1")
    parser.add_argument("--amplitude", type=float, default=90.0)
    parser.add_argument("--noise-std", type=float, default=2.0)
    parser.add_argument("--gap-us", type=int, default=80)
    parser.add_argument("--seed", type=int, default=1090)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()

    message = make_df17_callsign_message(args.icao, args.callsign)
    iq = synthesize_capture([message], args.amplitude, args.noise_std, args.gap_us, args.seed)
    write_iq_u8(args.output, iq)

    if not args.no_plots:
        plot_capture(iq, OUT_DIR / "figures")

    print(f"center_freq_hz={CENTER_FREQ_HZ}")
    print(f"sample_rate_hz={SAMPLE_RATE_HZ}")
    print(f"format=unsigned_u8_interleaved_iq")
    print(f"message_hex={message.hex().upper()}")
    print(f"icao={args.icao:06X}")
    print(f"callsign={args.callsign.upper().ljust(8)[:8]}")
    print(f"wrote={args.output}")


if __name__ == "__main__":
    main()
