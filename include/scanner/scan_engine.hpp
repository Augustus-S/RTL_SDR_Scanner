#pragma once

#include <atomic>
#include <complex>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>
#include <utility>
#include <vector>
#include "scanner/am_detector.hpp"
#include "scanner/fm_detector.hpp"
#include "scanner/persistent_async_reader.hpp"
#include "scanner/scan_profile.hpp"
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

    /** @brief Set the sweep frequency range in Hz. */
    void                                    setFreqRange(std::uint32_t startHz, std::uint32_t endHz);
    /** @brief Return the configured sweep frequency range in Hz. */
    std::pair<std::uint32_t, std::uint32_t> getFreqRange() const;
    /** @brief Set the scan speed/accuracy profile used for subsequent sweeps. */
    void                                    setScanProfile(ScanProfile profile);
    /** @brief Return the configured scan profile. */
    ScanProfile                             getScanProfile() const;
    /** @brief Request the scan sample rate in samples per second. */
    void                                    setScanSampleRate(std::uint32_t sampleRateHz);
    /** @brief Return the active sample rate, including any hardware fallback. */
    std::uint32_t                           getScanSampleRate() const;
    /** @brief Set when the RTL-SDR sample buffer should be reset between hops. */
    void                                    setResetPolicy(ResetPolicy policy);
    /** @brief Return the configured reset policy. */
    ResetPolicy                             getResetPolicy() const;

    enum class SweepResult {
        COMPLETED,
        STOPPED,
        DEVICE_ERROR
    };

    /**
     * @brief Execute one complete scan sweep using the current configuration.
     *
     * @param shouldContinue Optional cancellation predicate checked between hops.
     * @return COMPLETED when data was collected and pushed, STOPPED on
     * cancellation or invalid range, DEVICE_ERROR on unrecoverable SDR failure.
     * @note Changing range, profile, or active sample rate clears smoothing and
     * AM/FM detection tracks before the next sweep.
     */
    SweepResult doOneSweep(const std::function<bool()>& shouldContinue = {});

private:
    struct FmTrack {
        FmDetection detection;
        int         hits    = 0;
        int         misses  = 0;
        bool        visible = false;
    };

    struct AmTrack {
        AmDetection detection;
        int         hits    = 0;
        int         misses  = 0;
        bool        visible = false;
    };

    bool processOneHop(
        std::uint32_t                  centerFreq,
        int                            directSampling,
        const ScanProfileConfig&       profileConfig,
        std::vector<SegmentData>&      segments);
    void spliceAndPush(
        const std::vector<SegmentData>& segments,
        std::uint32_t                   sweepStartFreq,
        std::uint32_t                   sweepEndFreq,
        const std::function<bool()>&    shouldContinue);
    std::vector<FmDetection> updateFmTracks(std::vector<FmDetection> detections);
    std::vector<AmDetection> updateAmTracks(std::vector<AmDetection> detections);

    rtlsdr_dev_t*         dev_;
    rtl::tools::Pusher&   pusher_;
    rtl::tools::FftEngine fftEngine_;
    AMDetector            amDetector_;
    FMDetector            fmDetector_;

    std::atomic<bool>          running_{false};
    std::atomic<std::uint32_t> startFreq_{0};
    std::atomic<std::uint32_t> endFreq_{0};
    std::atomic<ScanProfile>   scanProfile_{ScanProfile::BALANCED};
    std::atomic<std::uint32_t> scanSampleRateHz_{2400000};
    std::atomic<ResetPolicy>   resetPolicy_{ResetPolicy::ADAPTIVE};
    std::uint32_t              activeSampleRateHz_{0};
    bool                       forceNextReset_{true};
    int                        sweepResetCount_{0};
    int                        lastDirectSampling_{-1};

    std::vector<std::uint8_t>              bufferU8_;
    std::vector<std::complex<short>>       bufferIQ_;
    std::vector<short>                     bufferQ_;
    std::vector<FmDetection>               currentFmDetections_;
    std::vector<AmDetection>               currentAmDetections_;
    int                                    currentFmIqBudget_{0};
    int                                    currentAmIqBudget_{0};
    std::vector<FmTrack>                   fmTracks_;
    std::vector<AmTrack>                   amTracks_;
    std::unique_ptr<PersistentAsyncReader> reader_;

    std::vector<double> prevSpectrum_;
    bool                hasPrevSpectrum_{false};
    bool                hasLastSweepConfig_{false};
    std::uint32_t       lastSweepStartFreq_{0};
    std::uint32_t       lastSweepEndFreq_{0};
    std::uint32_t       lastSweepSampleRateHz_{0};
    ScanProfile         lastSweepProfile_{ScanProfile::BALANCED};

    bool configureSampleRate(std::uint32_t requestedRateHz);
    void resetSweepHistory();
};

} // namespace rtl::scanner
