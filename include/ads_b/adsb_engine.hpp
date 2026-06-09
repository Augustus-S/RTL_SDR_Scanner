#pragma once

#include <rtl-sdr.h>
#include <atomic>
#include <chrono>
#include <functional>
#include <thread>
#include "ads_b/ads_b_demodulator.hpp"
#include "tools/pusher.hpp"

namespace rtl::sda_b {

class ADSBEngine {
public:
    ADSBEngine(rtlsdr_dev_t* dev, rtl::tools::Pusher& pusher, int maxGain);
    ~ADSBEngine();

    ADSBEngine(const ADSBEngine&)            = delete;
    ADSBEngine& operator=(const ADSBEngine&) = delete;

    enum class RunResult {
        COMPLETED,
        STOPPED,
        DEVICE_ERROR
    };

    RunResult runSlice(
        std::chrono::milliseconds    maxDuration    = std::chrono::milliseconds(0),
        const std::function<bool()>& shouldContinue = {});

    void requestStop();

    ADSBDemodulator& getDemodulator() {
        return demodulator_;
    }

    void removeStaleAircrafts(int ttlSec = 60);

private:
    rtlsdr_dev_t*     dev_;
    int               maxGain_;
    ADSBDemodulator   demodulator_;
    std::atomic<bool> stopRequested_{false};
};

} // namespace rtl::sda_b
