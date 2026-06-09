#pragma once

#include <atomic>
#include <complex>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>
#include <utility>
#include <vector>
#include "scanner/persistent_async_reader.hpp"
#include "scanner/types.hpp"
#include "tools/fft_engine.hpp"
#include "tools/pusher.hpp"

namespace rtl::scanner {

class ScanEngine {
public:
    ScanEngine(rtlsdr_dev_t* dev, rtl::tools::Pusher& pusher);
    ~ScanEngine();

    ScanEngine(const ScanEngine&)            = delete;
    ScanEngine& operator=(const ScanEngine&) = delete;

    void start();
    void stop();
    void requestStop();

    bool isRunning() const {
        return running_.load();
    }

    void                                    setFreqRange(std::uint32_t startHz, std::uint32_t endHz);
    std::pair<std::uint32_t, std::uint32_t> getFreqRange() const;

    enum class SweepResult {
        COMPLETED,
        STOPPED,
        DEVICE_ERROR
    };

    SweepResult doOneSweep(const std::function<bool()>& shouldContinue = {});

private:
    bool processOneHop(std::uint32_t centerFreq, int directSampling, std::vector<SegmentData>& segments);
    void spliceAndPush(
        const std::vector<SegmentData>& segments, std::uint32_t sweepStartFreq, std::uint32_t sweepEndFreq);

    rtlsdr_dev_t*         dev_;
    rtl::tools::Pusher&   pusher_;
    rtl::tools::FftEngine fftEngine_;

    std::atomic<bool>          running_{false};
    std::atomic<std::uint32_t> startFreq_{0};
    std::atomic<std::uint32_t> endFreq_{0};

    std::vector<std::uint8_t>              bufferU8_;
    std::vector<std::complex<short>>       bufferIQ_;
    std::vector<short>                     bufferQ_;
    std::unique_ptr<PersistentAsyncReader> reader_;
};

} // namespace rtl::scanner
