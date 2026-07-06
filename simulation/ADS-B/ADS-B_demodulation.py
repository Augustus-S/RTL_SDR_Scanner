"""Demodulate a synthetic ADS-B 1090ES unsigned 8-bit IQ capture."""

from __future__ import annotations

import argparse
import importlib.util
import os
import sys
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent / ".mplconfig"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

_MODULE_DIR = Path(__file__).resolve().parent


def _load_neighbor(module_name: str, filename: str):
    path = _MODULE_DIR / filename
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


_IQ_IO = _load_neighbor("mode_s_iq_io", "ADS-B_iq_io.py")
_MODULATION = _load_neighbor("mode_s_modulation", "ADS-B_modulation.py")

read_iq_u8 = _IQ_IO.read_iq_u8
MODE_S_CHARSET = _MODULATION.MODE_S_CHARSET
CENTER_FREQ_HZ = _MODULATION.CENTER_FREQ_HZ
LONG_MSG_BITS = _MODULATION.LONG_MSG_BITS
MODES_CHECKSUM_TABLE = _MODULATION.MODES_CHECKSUM_TABLE
PREAMBLE_PULSES = _MODULATION.PREAMBLE_PULSES
PREAMBLE_SAMPLES = _MODULATION.PREAMBLE_SAMPLES
SAMPLE_RATE_HZ = _MODULATION.SAMPLE_RATE_HZ
SAMPLES_PER_BIT = _MODULATION.SAMPLES_PER_BIT
SAMPLES_PER_US = _MODULATION.SAMPLES_PER_US

OUT_DIR = _MODULE_DIR
DEFAULT_INPUT = OUT_DIR / "ADS-B_1090ES.iq"


@dataclass(frozen=True)
class DecodedMessage:
    sample_index: int
    message: bytes
    crc_ok: bool
    df: int
    icao: int
    type_code: int
    callsign: str


def bytes_to_bits(data: bytes, bit_count: int | None = None) -> list[int]:
    bits: list[int] = []
    for value in data:
        bits.extend((value >> shift) & 1 for shift in range(7, -1, -1))
    return bits if bit_count is None else bits[:bit_count]


def pack_bits(bits: list[int]) -> bytes:
    out = bytearray((len(bits) + 7) // 8)
    for index, bit in enumerate(bits):
        out[index // 8] |= (bit & 1) << (7 - index % 8)
    return bytes(out)


def checksum(message: bytes) -> int:
    """Return zero when the 112-bit Mode-S message parity is valid."""
    bits = bytes_to_bits(message, 88)
    crc = 0
    for index, bit in enumerate(bits):
        if bit:
            crc ^= MODES_CHECKSUM_TABLE[index]
    received = int.from_bytes(message[-3:], "big")
    return (crc ^ received) & 0x00FFFFFF


def preamble_matches(magnitude: np.ndarray, index: int) -> bool:
    """Match the same 2 MS/s preamble shape used by the C++ decoder."""
    m = magnitude
    if index + PREAMBLE_SAMPLES + LONG_MSG_BITS * SAMPLES_PER_BIT >= m.size:
        return False
    j = index
    if not (
        m[j] > m[j + 1]
        and m[j + 1] < m[j + 2]
        and m[j + 2] > m[j + 3]
        and m[j + 3] < m[j]
        and m[j + 4] < m[j]
        and m[j + 5] < m[j]
        and m[j + 6] < m[j]
        and m[j + 7] > m[j + 8]
        and m[j + 8] < m[j + 9]
        and m[j + 9] > m[j + 6]
    ):
        return False

    high = (m[j] + m[j + 2] + m[j + 7] + m[j + 9]) / 6.0
    return not (
        m[j + 4] >= high
        or m[j + 5] >= high
        or m[j + 11] >= high
        or m[j + 12] >= high
        or m[j + 13] >= high
        or m[j + 14] >= high
    )


def decode_callsign(message: bytes) -> str:
    bits = bytes_to_bits(message[4:11], 56)
    chars: list[str] = []
    for offset in range(8, 56, 6):
        code = 0
        for bit in bits[offset : offset + 6]:
            code = (code << 1) | bit
        chars.append(MODE_S_CHARSET[code] if code < len(MODE_S_CHARSET) else "?")
    return "".join(chars).strip()


def decode_at(magnitude: np.ndarray, index: int) -> DecodedMessage:
    bits: list[int] = []
    data_start = index + PREAMBLE_SAMPLES
    for bit_index in range(LONG_MSG_BITS):
        first = magnitude[data_start + bit_index * SAMPLES_PER_BIT]
        second = magnitude[data_start + bit_index * SAMPLES_PER_BIT + 1]
        bits.append(1 if first > second else 0)

    message = pack_bits(bits)
    df = message[0] >> 3
    icao = int.from_bytes(message[1:4], "big")
    type_code = message[4] >> 3
    return DecodedMessage(
        sample_index=index,
        message=message,
        crc_ok=checksum(message) == 0,
        df=df,
        icao=icao,
        type_code=type_code,
        callsign=decode_callsign(message),
    )


def find_messages(magnitude: np.ndarray) -> list[DecodedMessage]:
    messages: list[DecodedMessage] = []
    index = 0
    frame_len = PREAMBLE_SAMPLES + LONG_MSG_BITS * SAMPLES_PER_BIT
    while index < magnitude.size - frame_len:
        if preamble_matches(magnitude, index):
            decoded = decode_at(magnitude, index)
            if decoded.crc_ok:
                messages.append(decoded)
                index += frame_len
                continue
        index += 1
    return messages


def plot_demodulation(magnitude: np.ndarray, decoded: DecodedMessage, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    start = decoded.sample_index
    data_start = start + PREAMBLE_SAMPLES
    frame_stop = data_start + 16 * SAMPLES_PER_BIT

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.stem(np.arange(start, start + PREAMBLE_SAMPLES + 16) / SAMPLES_PER_US, magnitude[start : start + PREAMBLE_SAMPLES + 16], basefmt=" ")
    for pulse in PREAMBLE_PULSES:
        ax.axvline((start + pulse) / SAMPLES_PER_US, color="tab:red", alpha=0.35, linewidth=1)
    ax.set_title("Detected ADS-B preamble")
    ax.set_xlabel("Time (us)")
    ax.set_ylabel("Magnitude")
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(output_dir / "03_detected_preamble.png", dpi=150)
    plt.close(fig)

    first = magnitude[data_start:frame_stop:SAMPLES_PER_BIT]
    second = magnitude[data_start + 1 : frame_stop + 1 : SAMPLES_PER_BIT]
    bit_axis = np.arange(first.size)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(bit_axis, first, marker="o", label="first half-bit")
    ax.plot(bit_axis, second, marker="o", label="second half-bit")
    ax.set_title("PPM half-bit energy comparison")
    ax.set_xlabel("Bit index")
    ax.set_ylabel("Magnitude")
    ax.legend()
    ax.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(output_dir / "04_ppm_bit_energy.png", dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Demodulate synthetic ADS-B 1090ES IQ data.")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()

    iq = read_iq_u8(args.input)
    magnitude = np.abs(iq)
    messages = find_messages(magnitude)

    print(f"center_freq_hz={CENTER_FREQ_HZ}")
    print(f"sample_rate_hz={SAMPLE_RATE_HZ}")
    print(f"input={args.input}")
    print(f"detected_messages={len(messages)}")

    for index, message in enumerate(messages, start=1):
        print(
            "message_{idx}=sample:{sample} df:{df} icao:{icao:06X} "
            "type_code:{tc} callsign:{callsign} crc_ok:{crc} hex:{hexmsg}".format(
                idx=index,
                sample=message.sample_index,
                df=message.df,
                icao=message.icao,
                tc=message.type_code,
                callsign=message.callsign,
                crc=message.crc_ok,
                hexmsg=message.message.hex().upper(),
            )
        )

    if messages and not args.no_plots:
        plot_demodulation(magnitude, messages[0], OUT_DIR / "figures")

    if not messages:
        raise SystemExit("No valid ADS-B messages detected")


if __name__ == "__main__":
    main()
