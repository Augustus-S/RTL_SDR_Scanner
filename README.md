# RTL_SDR_Scanner

RTL_SDR_Scanner is a C++20 RTL-SDR spectrum scanner with optional ADS-B / Mode-S decoding. It can sweep a configurable frequency range, post spectrum data to an HTTP receiver, and publish decoded aircraft states.

## Features

- Spectrum sweep with FFTW-based power calculation and spectrum stitching.
- ADS-B decoding from 1090 MHz RTL-SDR IQ samples.
- HTTP data push for scan spectra and ADS-B aircraft lists.
- HTTP control server for runtime scan / ADS-B enable flags and scan parameters.
- Namespaced C++ interfaces split by responsibility:
  - `rtl::scanner`: SDR device access, persistent reader, scan data models.
  - `rtl::sda_b`: ADS-B aircraft model and decoder.
  - `rtl::tools`: logging, HTTP, DSP helpers and async pusher.

## Dependencies

Runtime and build dependencies:

- CMake 3.16+
- C++20 compiler
- `librtlsdr`
- `fftw3`
- `spdlog`
- `nlohmann_json`
- POSIX threads

Example package names on Debian/Ubuntu-like systems:

```bash
sudo apt install cmake g++ librtlsdr-dev libfftw3-dev libspdlog-dev nlohmann-json3-dev
```

## Build

```bash
cmake -S . -B build
cmake --build build
```

The executable is generated as:

```bash
build/RTL_SDR_Scanner
```

## Usage

Show help:

```bash
build/RTL_SDR_Scanner --help
```

ADS-B only:

```bash
build/RTL_SDR_Scanner --mode 1
```

Scan only:

```bash
build/RTL_SDR_Scanner --mode 2 --start-freq 88 --end-freq 108
```

ADS-B plus scan time slicing:

```bash
build/RTL_SDR_Scanner --mode 3 --start-freq 100 --end-freq 160
```

Frequencies passed on the command line are in MHz. Internally, public C++ APIs use Hz unless documented otherwise.

## HTTP Data Output

The scanner posts JSON data to:

```text
http://127.0.0.1:23568/api/service
```

Python helper receivers are available:

```bash
python3 python/receiver_raw.py
python3 python/receiver_plot.py
```

## HTTP Control API

The control server listens on:

```text
0.0.0.0:23569
```

Endpoints:

- `POST /adsb/start`
- `POST /adsb/stop`
- `POST /scan/start`
- `POST /scan/stop`
- `POST /scan/param`
- `GET /status`

Example scan parameter update:

```bash
curl -X POST http://127.0.0.1:23569/scan/param \
  -H 'Content-Type: application/json' \
  -d '{"start_freq": 88000000, "end_freq": 108000000}'
```

`/scan/param` expects Hz, not MHz.

## Public C++ Interfaces

### Constants

Shared constants are defined in:

```cpp
#include "constants.hpp"
```

Namespace:

```cpp
rtl::constants
```

Examples: `SCAN_SAMPLE_RATE`, `ADSB_FREQ`, `CONTROL_PORT`, `FFT_SIZE`.

### Scanner Module

Headers:

```cpp
#include "scanner/types.hpp"
#include "scanner/rtl_sdr_device.hpp"
#include "scanner/persistent_async_reader.hpp"
```

Namespace:

```cpp
rtl::scanner
```

Primary types:

- `ScanPlan`
- `SegmentData`
- `RtlSdrDevice`
- `PersistentAsyncReader`

`PersistentAsyncReader::read()` uses a caller-owned output buffer. `outLen` is both input capacity and output bytes copied.

### ADS-B Module

Headers:

```cpp
#include "ads_b/aircraft.hpp"
#include "ads_b/ads_b_demodulator.hpp"
```

Namespace:

```cpp
rtl::sda_b
```

Primary types:

- `Aircraft`
- `ADSBDemodulator`

`ADSBDemodulator::processIq()` expects interleaved unsigned 8-bit IQ bytes from librtlsdr.

### Tools Module

Headers:

```cpp
#include "tools/tools.hpp"
#include "tools/pusher.hpp"
```

Namespace:

```cpp
rtl::tools
```

Primary APIs:

- `initSpdlog()`
- `post()`
- `get()`
- `removeDc()`
- `calculateFft()`
- `spectrumToDb()`
- `spliceSpectrum()`
- `detectPeaks()`
- `Pusher`

## Notes

- Running against RTL-SDR hardware may require udev rules or root privileges.
- Only one process should own the RTL-SDR device at a time.
- ADS-B decoding is tuned for 1090 MHz at 2 MS/s.
- The control API binds to all interfaces. Use firewall rules or change the bind address before exposing it on untrusted networks.
- `rtl::tools::getCmdResult()` invokes the shell and must not receive untrusted input.
- The namespace `rtl::sda_b` follows the current project request. The folder name remains `ads_b`.
