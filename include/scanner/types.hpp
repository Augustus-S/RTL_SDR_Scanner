#pragma once

#include <vector>

/**
 * @file types.hpp
 * @brief Data models used by the spectrum scanner.
 */

namespace rtl::scanner {

/**
 * @brief Frequency sweep plan derived from a scan range and sample rate.
 *
 * @note Frequencies and rate are expressed in Hz. The plan does not touch
 * hardware; it only describes how many hops and bins are expected.
 */
struct ScanPlan {
    double              startFreq;    /**< Sweep start frequency in Hz. */
    double              endFreq;      /**< Sweep end frequency in Hz. */
    double              rate;         /**< RTL-SDR sample rate in samples per second. */
    int                 numSteps;     /**< Number of scan hops. */
    int                 displayBins;  /**< Number of bins intended for final display. */
    std::vector<double> centerFreqs;  /**< Per-hop center frequencies in Hz. */
    double              freqPerBin;   /**< Frequency resolution per FFT bin in Hz. */
    int                 overlapBins;  /**< FFT bins overlapped between adjacent hops. */
    int                 usablePerHop; /**< FFT bins consumed from each hop after overlap. */
};

/**
 * @brief One FFT segment captured around a single center frequency.
 */
struct SegmentData {
    std::vector<double> spectrum;   /**< Spectrum power values in dBFS. */
    double              centerFreq; /**< Center frequency of this segment in Hz. */
};

} // namespace rtl::scanner
