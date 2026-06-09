#pragma once

#include <rtl-sdr.h>
#include <cstdint>
#include <string>
#include <vector>
#include "constants.hpp"

/**
 * @file rtl_sdr_device.hpp
 * @brief RAII wrapper around librtlsdr device handles.
 */

namespace rtl::scanner {

/**
 * @brief Owns one RTL-SDR device handle.
 *
 * @note This wrapper is non-copyable and movable. Methods log librtlsdr errors
 * but generally keep the device open unless the underlying close operation runs.
 */
class RtlSdrDevice {
public:
    /**
     * @brief Open an RTL-SDR device by index.
     * @param deviceIndex Zero-based librtlsdr device index.
     * @note If opening fails, isOpen() returns false.
     */
    explicit RtlSdrDevice(std::uint32_t deviceIndex = 0);

    /**
     * @brief Close the device if it is open.
     */
    ~RtlSdrDevice();

    RtlSdrDevice(const RtlSdrDevice&)            = delete;
    RtlSdrDevice& operator=(const RtlSdrDevice&) = delete;

    RtlSdrDevice(RtlSdrDevice&& other) noexcept;
    RtlSdrDevice& operator=(RtlSdrDevice&& other) noexcept;

    /** @brief Get the number of available RTL-SDR devices. */
    static int getDeviceCount();

    /** @brief Set sample rate in samples per second. */
    void setSampleRate(std::uint32_t rate);

    /** @brief Set center frequency in Hz and adjust direct sampling mode. */
    void setCenterFreq(std::uint32_t freq);

    /** @brief Set direct sampling mode (0=off, 1=I-ADC, 2=Q-ADC). */
    void setDirectSampling(int mode);

    /** @brief Get current direct sampling mode. */
    int getDirectSampling() const;

    /** @brief Set tuner gain in tenths of dB, or negative for AGC. */
    void setGain(int gain);

    /** @brief Select the maximum gain reported by the tuner. */
    void setMaxGain();

    /** @brief Get the list of available tuner gain values. */
    std::vector<int> getTunerGains() const;

    /** @brief Get the current tuner gain value. */
    int getCurrentGain() const;

    /** @brief Enable or disable tuner AGC mode. */
    void setAgcMode(bool enable);

    /** @brief Set oscillator frequency correction in PPM. */
    void setFreqCorrection(int ppm);

    /**
     * @brief Read samples synchronously.
     * @param buffer Output byte buffer.
     * @param bufLen Buffer length in bytes.
     * @param nRead Output number of bytes read.
     * @return librtlsdr status code.
     */
    int readSync(std::uint8_t* buffer, int bufLen, int* nRead);

    /** @brief Reset the device sample buffer. */
    void resetBuffer();

    /**
     * @brief Stabilize the device by performing dummy reads.
     * @param dummyReads Number of dummy read iterations.
     * @param bufSize Size of each dummy read buffer in bytes.
     */
    void stabilize(int dummyReads = 100, int bufSize = 16384);

    /** @brief Get the device name. */
    std::string getDeviceName() const;

    /** @return true when a device handle is open. */
    bool isOpen() const {
        return dev_ != nullptr;
    }

    /** @return Raw librtlsdr device pointer. The wrapper retains ownership. */
    rtlsdr_dev_t* getRawDev() const {
        return dev_;
    }

private:
    rtlsdr_dev_t* dev_                   = nullptr;
    int           currentDirectSampling_ = 0;
    int           currentGain_           = 0;
};

} // namespace rtl::scanner