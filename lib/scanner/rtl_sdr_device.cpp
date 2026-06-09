#include "scanner/rtl_sdr_device.hpp"
#include "constants.hpp"
#include <spdlog/spdlog.h>

namespace rtl::scanner {

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
    , currentDirectSampling_(other.currentDirectSampling_) {
    other.dev_ = nullptr;
}

RtlSdrDevice& RtlSdrDevice::operator=(RtlSdrDevice&& other) noexcept {
    if (this != &other) {
        if (dev_) { rtlsdr_close(dev_); }
        dev_                   = other.dev_;
        currentDirectSampling_ = other.currentDirectSampling_;
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

void RtlSdrDevice::setGain(int gain) {
    if (!dev_) return;

    if (gain < 0) {
        rtlsdr_set_tuner_gain_mode(dev_, 0);
    } else {
        rtlsdr_set_tuner_gain_mode(dev_, 1);
        rtlsdr_set_tuner_gain(dev_, gain);
    }
}

void RtlSdrDevice::setMaxGain() {
    if (!dev_) return;

    int gains[100] = {};
    int numGains   = rtlsdr_get_tuner_gains(dev_, gains);
    if (numGains <= 0) {
        spdlog::error("Failed to get tuner gains");
        return;
    }

    int maxGain = gains[numGains - 1];
    rtlsdr_set_tuner_gain_mode(dev_, 1);
    rtlsdr_set_tuner_gain(dev_, maxGain);
    spdlog::info("Max available gain: {} dB", maxGain / 10.0);
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

} // namespace rtl::scanner
