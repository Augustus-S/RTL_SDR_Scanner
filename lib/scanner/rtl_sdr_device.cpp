#include "scanner/rtl_sdr_device.hpp"
#include "constants.hpp"
#include <spdlog/spdlog.h>
#include <vector>
#include <cstring>

namespace rtl::scanner {

int RtlSdrDevice::getDeviceCount() {
    return rtlsdr_get_device_count();
}

RtlSdrDevice::RtlSdrDevice(uint32_t deviceIndex) {
    int deviceCount = rtlsdr_get_device_count();
    if (deviceCount == 0) {
        spdlog::error("No RTL-SDR devices found");
        return;
    }

    if (deviceIndex >= static_cast<uint32_t>(deviceCount)) {
        spdlog::error("Device index {} out of range ({} devices found)", deviceIndex, deviceCount);
        return;
    }

    int ret = rtlsdr_open(&dev_, deviceIndex);
    if (ret < 0) {
        spdlog::error("Failed to open RTL-SDR device {}: error {}", deviceIndex, ret);
        dev_ = nullptr;
        return;
    }

    spdlog::info("Opened RTL-SDR device: {}", rtlsdr_get_device_name(deviceIndex));
}

RtlSdrDevice::~RtlSdrDevice() {
    if (dev_) {
        rtlsdr_close(dev_);
        dev_ = nullptr;
    }
}

RtlSdrDevice::RtlSdrDevice(RtlSdrDevice&& other) noexcept
    : dev_(other.dev_)
    , currentDirectSampling_(other.currentDirectSampling_)
    , currentGain_(other.currentGain_) {
    other.dev_ = nullptr;
}

RtlSdrDevice& RtlSdrDevice::operator=(RtlSdrDevice&& other) noexcept {
    if (this != &other) {
        if (dev_) { rtlsdr_close(dev_); }
        dev_                   = other.dev_;
        currentDirectSampling_ = other.currentDirectSampling_;
        currentGain_           = other.currentGain_;
        other.dev_             = nullptr;
    }
    return *this;
}

void RtlSdrDevice::setSampleRate(uint32_t rate) {
    if (!dev_) return;
    int ret = rtlsdr_set_sample_rate(dev_, rate);
    if (ret < 0) { spdlog::error("Failed to set sample rate {}: error {}", rate, ret); }
}

void RtlSdrDevice::setCenterFreq(uint32_t freq) {
    if (!dev_) return;

    int needDs = (freq < rtl::constants::LOW_FREQ_THRESHOLD) ? 2 : 0;
    if (needDs != currentDirectSampling_) {
        int ret = rtlsdr_set_direct_sampling(dev_, needDs);
        if (ret < 0) {
            spdlog::error("Failed to set direct sampling mode {}: error {}", needDs, ret);
        } else {
            currentDirectSampling_ = needDs;
        }
    }

    int ret = rtlsdr_set_center_freq(dev_, freq);
    if (ret < 0) { spdlog::error("Failed to set center freq {}: error {}", freq, ret); }
}

void RtlSdrDevice::setDirectSampling(int mode) {
    if (!dev_) return;
    int ret = rtlsdr_set_direct_sampling(dev_, mode);
    if (ret < 0) {
        spdlog::error("Failed to set direct sampling mode {}: error {}", mode, ret);
    } else {
        currentDirectSampling_ = mode;
    }
}

int RtlSdrDevice::getDirectSampling() const {
    return currentDirectSampling_;
}

void RtlSdrDevice::setGain(int gain) {
    if (!dev_) return;

    if (gain < 0) {
        rtlsdr_set_tuner_gain_mode(dev_, 0);
    } else {
        rtlsdr_set_tuner_gain_mode(dev_, 1);
        int ret = rtlsdr_set_tuner_gain(dev_, gain);
        if (ret < 0) {
            spdlog::error("Failed to set tuner gain {}: error {}", gain, ret);
        } else {
            currentGain_ = gain;
        }
    }
}

void RtlSdrDevice::setMaxGain() {
    if (!dev_) return;

    auto gains = getTunerGains();
    if (gains.empty()) {
        spdlog::error("Failed to get tuner gains");
        return;
    }

    int maxGain = gains.back();
    rtlsdr_set_tuner_gain_mode(dev_, 1);
    int ret = rtlsdr_set_tuner_gain(dev_, maxGain);
    if (ret < 0) {
        spdlog::error("Failed to set max tuner gain {}: error {}", maxGain, ret);
    } else {
        currentGain_ = maxGain;
        spdlog::info("Max available gain: {} dB", maxGain / 10.0);
    }
}

std::vector<int> RtlSdrDevice::getTunerGains() const {
    if (!dev_) return {};

    int numGains = rtlsdr_get_tuner_gains(dev_, nullptr);
    if (numGains <= 0) return {};

    std::vector<int> gains(static_cast<std::size_t>(numGains));
    int              ret = rtlsdr_get_tuner_gains(dev_, gains.data());
    if (ret <= 0) return {};
    gains.resize(static_cast<std::size_t>(ret));
    return gains;
}

int RtlSdrDevice::getCurrentGain() const {
    return currentGain_;
}

void RtlSdrDevice::setAgcMode(bool enable) {
    if (!dev_) return;
    rtlsdr_set_agc_mode(dev_, enable ? 1 : 0);
}

void RtlSdrDevice::setFreqCorrection(int ppm) {
    if (!dev_) return;
    int ret = rtlsdr_set_freq_correction(dev_, ppm);
    if (ret < 0 && ret != -2) { spdlog::error("Failed to set freq correction {}: error {}", ppm, ret); }
}

int RtlSdrDevice::readSync(uint8_t* buffer, int bufLen, int* nRead) {
    if (!dev_) return -1;
    return rtlsdr_read_sync(dev_, buffer, bufLen, nRead);
}

void RtlSdrDevice::resetBuffer() {
    if (!dev_) return;
    rtlsdr_reset_buffer(dev_);
}

void RtlSdrDevice::stabilize(int dummyReads, int bufSize) {
    if (!dev_) return;

    resetBuffer();
    std::vector<uint8_t> dummyBuf(bufSize);
    int                  dummyLen = 0;
    for (int i = 0; i < dummyReads; ++i) { rtlsdr_read_sync(dev_, dummyBuf.data(), bufSize, &dummyLen); }
    spdlog::info("Device stabilized");
}

std::string RtlSdrDevice::getDeviceName() const {
    if (!dev_) return "";
    const char* name = rtlsdr_get_device_name(0);
    return name ? std::string(name) : std::string();
}

} // namespace rtl::scanner
