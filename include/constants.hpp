#pragma once

#include <cstdint>

/**
 * @file constants.hpp
 * @brief Shared compile-time constants for RTL-SDR scanning and ADS-B decoding.
 */

namespace rtl::constants {

/** @brief FFT size used by the spectrum scanner. */
inline constexpr int FFT_SIZE = 2048;

/** @brief Default RTL-SDR byte buffer length for scan and ADS-B reads. */
inline constexpr int BUFFER_LEN = 16 * 16384;

/** @brief Frequency below which direct sampling mode is used. Unit: Hz. */
inline constexpr std::uint32_t LOW_FREQ_THRESHOLD = 25000000;

/** @brief Maximum unsigned 8-bit ADC magnitude after DC centering. */
inline constexpr double ADC_MAX_VAL = 127.0;

/** @brief Full-scale reference power for dBFS conversion. */
inline constexpr double FULL_SCALE_REF = ADC_MAX_VAL * ADC_MAX_VAL;

/** @brief Default scan sample rate. Unit: samples per second. */
inline constexpr std::uint32_t SCAN_SAMPLE_RATE = 2000000;

/** @brief Lowest allowed scan frequency. Unit: Hz. */
inline constexpr std::uint32_t MIN_FREQ = 10000000;

/** @brief Highest allowed scan frequency. Unit: Hz. */
inline constexpr std::uint32_t MAX_FREQ = 1070000000;

/** @brief Maximum allowed scan bandwidth. Unit: Hz. */
inline constexpr std::uint32_t MAX_BANDWIDTH = 100000000;

/** @brief Scan hop step. Unit: Hz. */
inline constexpr std::uint32_t STEP_FREQ = SCAN_SAMPLE_RATE / 2;

/** @brief Default data push URL. */
inline constexpr const char* DATA_URL = "http://127.0.0.1:23568/api/service";

/** @brief ADS-B sample rate. Unit: samples per second. */
inline constexpr std::uint32_t ADSB_SAMPLE_RATE = 2000000;

/** @brief ADS-B center frequency. Unit: Hz. */
inline constexpr std::uint32_t ADSB_FREQ = 1090000000;

/** @brief RTL-SDR async read block length for ADS-B. Unit: bytes. */
inline constexpr std::uint32_t ADSB_READ_LEN = 16 * 16384;

/** @brief Synchronous scanner read timeout. Unit: milliseconds. */
inline constexpr int READ_TIMEOUT_MS = 3000;

/** @brief HTTP control server port. */
inline constexpr int CONTROL_PORT = 23569;

} // namespace rtl::constants
