#include "ads_b/adsb_engine.hpp"
#include "constants.hpp"
#include <spdlog/spdlog.h>
#include <thread>
#include <chrono>

namespace rtl::sda_b {

namespace {

void adsbAsyncCallback(unsigned char* buf, std::uint32_t len, void* ctx) {
    auto* demod = static_cast<ADSBDemodulator*>(ctx);
    demod->processIq(buf, len);
}

} // namespace

ADSBEngine::ADSBEngine(rtlsdr_dev_t* dev, rtl::tools::Pusher& pusher, int maxGain)
    : dev_(dev)
    , maxGain_(maxGain)
    , demodulator_(pusher) {
}

ADSBEngine::~ADSBEngine() = default;

void ADSBEngine::runSlice(std::chrono::milliseconds maxDuration) {
    stopRequested_ = false;

    rtlsdr_set_sample_rate(dev_, rtl::constants::ADSB_SAMPLE_RATE);
    rtlsdr_set_direct_sampling(dev_, 0);
    rtlsdr_set_tuner_gain_mode(dev_, 1);
    rtlsdr_set_tuner_gain(dev_, maxGain_);
    rtlsdr_set_center_freq(dev_, rtl::constants::ADSB_FREQ);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    rtlsdr_reset_buffer(dev_);

    std::atomic<int> adsbAsyncRet{0};
    std::thread      adsbThread([&]() {
        int ret = rtlsdr_read_async(dev_, adsbAsyncCallback, &demodulator_, 12, rtl::constants::ADSB_READ_LEN);
        adsbAsyncRet.store(ret);
    });

    auto adsbStart = std::chrono::steady_clock::now();
    while (!stopRequested_.load()) {
        if (maxDuration.count() > 0 && std::chrono::steady_clock::now() - adsbStart >= maxDuration) break;
        if (adsbAsyncRet.load() != 0) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (adsbAsyncRet.load() < 0) {
        spdlog::error("rtlsdr_read_async failed: {}", adsbAsyncRet.load());
    } else if (adsbAsyncRet.load() == 0) {
        rtlsdr_cancel_async(dev_);
    }
    if (adsbThread.joinable()) adsbThread.join();
}

void ADSBEngine::requestStop() {
    stopRequested_ = true;
}

void ADSBEngine::removeStaleAircrafts(int ttlSec) {
    demodulator_.removeStaleAircrafts(ttlSec);
}

} // namespace rtl::sda_b