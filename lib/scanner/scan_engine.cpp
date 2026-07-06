#include "scanner/scan_engine.hpp"
#include "constants.hpp"
#include "tools/tools.hpp"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <nlohmann/json.hpp>

/**
 * @file scan_engine.cpp
 * @brief Frequency sweep engine, spectrum stitching, and AM/FM detection output.
 */

namespace rtl::scanner {

namespace {

double smoothValue(double previous, double current, double alpha = 0.45) {
    return previous * (1.0 - alpha) + current * alpha;
}

void smoothDetection(FmDetection& target, const FmDetection& current) {
    target.startFreqHz = smoothValue(target.startFreqHz, current.startFreqHz);
    target.endFreqHz   = smoothValue(target.endFreqHz, current.endFreqHz);
    target.centerHz    = smoothValue(target.centerHz, current.centerHz);
    target.bandwidthHz = smoothValue(target.bandwidthHz, current.bandwidthHz);
    target.peakDb      = smoothValue(target.peakDb, current.peakDb);
    target.avgDb       = smoothValue(target.avgDb, current.avgDb);
    target.noiseDb     = smoothValue(target.noiseDb, current.noiseDb);
    target.snrDb       = smoothValue(target.snrDb, current.snrDb);
    target.confidence  = smoothValue(target.confidence, current.confidence);
    target.pilotSnrDb  = smoothValue(target.pilotSnrDb, current.pilotSnrDb);
    target.audioSnrDb  = smoothValue(target.audioSnrDb, current.audioSnrDb);
    target.deviationHz = smoothValue(target.deviationHz, current.deviationHz);
    target.paprDb      = smoothValue(target.paprDb, current.paprDb);
    target.amVariance  = smoothValue(target.amVariance, current.amVariance);
    target.fmRmsHz     = smoothValue(target.fmRmsHz, current.fmRmsHz);
    target.verified    = current.verified || target.verified;
    target.stereo      = current.stereo || target.stereo;
    target.rds         = current.rds || target.rds;
}

void smoothDetection(AmDetection& target, const AmDetection& current) {
    target.startFreqHz     = smoothValue(target.startFreqHz, current.startFreqHz);
    target.endFreqHz       = smoothValue(target.endFreqHz, current.endFreqHz);
    target.centerHz        = smoothValue(target.centerHz, current.centerHz);
    target.bandwidthHz     = smoothValue(target.bandwidthHz, current.bandwidthHz);
    target.peakDb          = smoothValue(target.peakDb, current.peakDb);
    target.avgDb           = smoothValue(target.avgDb, current.avgDb);
    target.noiseDb         = smoothValue(target.noiseDb, current.noiseDb);
    target.snrDb           = smoothValue(target.snrDb, current.snrDb);
    target.confidence      = smoothValue(target.confidence, current.confidence);
    target.audioSnrDb      = smoothValue(target.audioSnrDb, current.audioSnrDb);
    target.modulationDepth = smoothValue(target.modulationDepth, current.modulationDepth);
    target.carrierSnrDb    = smoothValue(target.carrierSnrDb, current.carrierSnrDb);
    target.fmRmsHz         = smoothValue(target.fmRmsHz, current.fmRmsHz);
    target.verified        = current.verified || target.verified;
}

} // namespace

ScanEngine::ScanEngine(rtlsdr_dev_t* dev, rtl::tools::Pusher& pusher)
    : dev_(dev)
    , pusher_(pusher)
    , fftEngine_(rtl::constants::FFT_SIZE) {
    bufferU8_.resize(rtl::constants::BUFFER_LEN);
    bufferIQ_.resize(rtl::constants::BUFFER_LEN / 2);
    bufferQ_.resize(rtl::constants::BUFFER_LEN / 2);
}

ScanEngine::~ScanEngine() {
    stop();
}

void ScanEngine::start() {
    if (running_.exchange(true)) return;

    const auto profileConfig = configForProfile(scanProfile_.load(), scanSampleRateHz_.load());
    configureSampleRate(profileConfig.sampleRateHz);
    forceNextReset_ = true;
    reader_ = std::make_unique<PersistentAsyncReader>(dev_);
}

void ScanEngine::stop() {
    if (!running_.exchange(false)) return;

    if (reader_) {
        reader_->shutdown();
        reader_.reset();
    }
}

void ScanEngine::requestStop() {
    running_ = false;
}

void ScanEngine::setFreqRange(std::uint32_t startHz, std::uint32_t endHz) {
    startFreq_.store(startHz);
    endFreq_.store(endHz);
}

std::pair<std::uint32_t, std::uint32_t> ScanEngine::getFreqRange() const {
    return {startFreq_.load(), endFreq_.load()};
}

void ScanEngine::setScanProfile(ScanProfile profile) {
    scanProfile_.store(profile);
}

ScanProfile ScanEngine::getScanProfile() const {
    return scanProfile_.load();
}

void ScanEngine::setScanSampleRate(std::uint32_t sampleRateHz) {
    scanSampleRateHz_.store(sampleRateHz);
}

std::uint32_t ScanEngine::getScanSampleRate() const {
    return activeSampleRateHz_ != 0 ? activeSampleRateHz_ : scanSampleRateHz_.load();
}

void ScanEngine::setResetPolicy(ResetPolicy policy) {
    resetPolicy_.store(policy);
}

ResetPolicy ScanEngine::getResetPolicy() const {
    return resetPolicy_.load();
}

void ScanEngine::resetSweepHistory() {
    // History is tied to the scan frequency grid. Reusing it after a range,
    // profile, or sample-rate change would smear old spectrum/detections into a
    // different set of bins.
    prevSpectrum_.clear();
    hasPrevSpectrum_ = false;
    fmTracks_.clear();
    amTracks_.clear();
    currentFmDetections_.clear();
    currentAmDetections_.clear();
    forceNextReset_     = true;
    lastDirectSampling_ = -1;
}

bool ScanEngine::configureSampleRate(std::uint32_t requestedRateHz) {
    std::vector<std::uint32_t> candidates;
    candidates.push_back(requestedRateHz);
    if (requestedRateHz > 2400000) candidates.push_back(2400000);
    if (requestedRateHz > 2000000) candidates.push_back(2000000);

    // Some dongles reject the fastest profile rates. Fall back in descending
    // order while publishing the active rate back through getScanSampleRate().
    for (auto rate : candidates) {
        if (rate == activeSampleRateHz_) return true;
        const int ret = rtlsdr_set_sample_rate(dev_, rate);
        if (ret == 0) {
            if (rate != requestedRateHz) {
                spdlog::warn(
                    "Requested scan sample rate {} MS/s failed, using fallback {} MS/s",
                    requestedRateHz / 1e6,
                    rate / 1e6);
            }
            activeSampleRateHz_ = rate;
            scanSampleRateHz_.store(rate);
            forceNextReset_ = true;
            lastDirectSampling_ = -1;
            return true;
        }
        spdlog::warn("Failed to set scan sample rate {} MS/s: error {}", rate / 1e6, ret);
    }
    return false;
}

ScanEngine::SweepResult ScanEngine::doOneSweep(const std::function<bool()>& shouldContinue) {
    if (!running_ || !reader_) return SweepResult::STOPPED;

    std::uint32_t sweep_start_freq = startFreq_.load();
    std::uint32_t sweep_end_freq   = endFreq_.load();
    if (sweep_end_freq <= sweep_start_freq) {
        spdlog::warn("Invalid scan range: {} - {} Hz", sweep_start_freq, sweep_end_freq);
        return SweepResult::STOPPED;
    }

    auto          profileConfig    = configForProfile(scanProfile_.load(), scanSampleRateHz_.load());
    if (!configureSampleRate(profileConfig.sampleRateHz)) {
        running_ = false;
        return SweepResult::DEVICE_ERROR;
    }
    profileConfig = configForProfile(scanProfile_.load(), activeSampleRateHz_);
    const auto activeProfile = profileConfig.profile;

    const bool configChanged =
        !hasLastSweepConfig_ || lastSweepStartFreq_ != sweep_start_freq || lastSweepEndFreq_ != sweep_end_freq
        || lastSweepSampleRateHz_ != activeSampleRateHz_ || lastSweepProfile_ != activeProfile;
    if (configChanged) {
        // A new scan grid invalidates smoothing and track memory even if the
        // resulting stitched spectrum happens to have the same number of bins.
        resetSweepHistory();
        hasLastSweepConfig_    = true;
        lastSweepStartFreq_    = sweep_start_freq;
        lastSweepEndFreq_      = sweep_end_freq;
        lastSweepSampleRateHz_ = activeSampleRateHz_;
        lastSweepProfile_      = activeProfile;
    }

    auto                     sweep_start = std::chrono::steady_clock::now();
    std::vector<SegmentData> segments;
    currentFmDetections_.clear();
    currentAmDetections_.clear();
    currentFmIqBudget_ = profileConfig.maxFmIqVerifications;
    currentAmIqBudget_ = profileConfig.maxAmIqVerifications;
    sweepResetCount_ = 0;
    int hopCount = 0;

    // Each hop is centered at the current sweep frequency and covers roughly one
    // sample-rate-wide slice; profile step controls overlap versus speed.
    for (std::uint32_t cur_freq = sweep_start_freq;
         cur_freq <= sweep_end_freq && running_ && (!shouldContinue || shouldContinue());
         cur_freq += profileConfig.stepHz) {
        int need_ds = (cur_freq < rtl::constants::LOW_FREQ_THRESHOLD) ? 2 : 0;
        if (!processOneHop(cur_freq, need_ds, profileConfig, segments)) {
            return running_ ? SweepResult::DEVICE_ERROR : SweepResult::STOPPED;
        }
        ++hopCount;
    }

    if (!running_ || (shouldContinue && !shouldContinue())) return SweepResult::STOPPED;

    if (!segments.empty()) {
        spliceAndPush(segments, sweep_start_freq, sweep_end_freq, shouldContinue);

        auto sweep_end     = std::chrono::steady_clock::now();
        auto sweep_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(sweep_end - sweep_start);
        const auto elapsedMs = sweep_elapsed.count();
        const auto avgHopMs  = hopCount > 0 ? static_cast<double>(elapsedMs) / static_cast<double>(hopCount) : 0.0;
        spdlog::info(
            "Sweep complete: profile={}, sample_rate={} MS/s, step={} MHz, settle={} ms, reset_policy={}, resets={}, hops={}, took={} ms, avg_hop={:.1f} ms, fm={}, am={}",
            profileConfig.name,
            activeSampleRateHz_ / 1e6,
            profileConfig.stepHz / 1e6,
            profileConfig.tuneSettleMs,
            resetPolicyName(resetPolicy_.load()),
            sweepResetCount_,
            hopCount,
            elapsedMs,
            avgHopMs,
            currentFmDetections_.size(),
            currentAmDetections_.size());
    }

    return SweepResult::COMPLETED;
}

bool ScanEngine::processOneHop(
    std::uint32_t                  centerFreq,
    int                            directSampling,
    const ScanProfileConfig&       profileConfig,
    std::vector<SegmentData>&      segments) {
    std::uint32_t n_read = 4 * rtl::constants::FFT_SIZE * 2;
    const bool forceReset            = forceNextReset_;
    const bool directSamplingChanged = directSampling != lastDirectSampling_;
    const auto resetPolicy           = resetPolicy_.load();
    if (resetPolicy == ResetPolicy::ALWAYS || forceReset || directSamplingChanged) {
        ++sweepResetCount_;
    }
    // Read one short IQ block after hardware tuning/settling. The reader owns
    // direct-sampling switches and buffer resets so the sweep loop stays linear.
    auto          scan_result = reader_->read(
        bufferU8_.data(),
        &n_read,
        centerFreq,
        directSampling,
        rtl::constants::READ_TIMEOUT_MS,
        profileConfig.tuneSettleMs,
        resetPolicy,
        forceReset);
    forceNextReset_ = false;

    if (scan_result != PersistentAsyncReader::ReadResult::SUCCESS) {
        forceNextReset_ = true;
        if (scan_result == PersistentAsyncReader::ReadResult::DEVICE_ERROR) {
            spdlog::error("Device error @ {} MHz, aborting", centerFreq / 1e6);
            running_ = false;
            return false;
        } else {
            spdlog::warn("Read timeout @ {} MHz", centerFreq / 1e6);
        }
        return true;
    }
    lastDirectSampling_ = directSampling;

    int samples = static_cast<int>(n_read / 2);
    samples     = std::min<int>(samples, static_cast<int>(bufferIQ_.size()));
    for (int i = 0; i < samples; ++i) {
        short I      = static_cast<short>(bufferU8_[2 * i]) - 127;
        short Q      = static_cast<short>(bufferU8_[2 * i + 1]) - 127;
        bufferIQ_[i] = std::complex<short>(I, Q);
        bufferQ_[i]  = Q;
    }

    // FFT power uses centered complex IQ. Direct-sampling RSSI later uses the Q
    // channel directly because only one ADC path carries the low-frequency signal.
    rtl::tools::removeDc(bufferIQ_.data(), samples);

    auto [fft_power_sum, groups_num] = fftEngine_.accumulatePower(bufferIQ_.data(), samples);

    if (groups_num <= 0) return true;

    double rssi;
    if (centerFreq < rtl::constants::LOW_FREQ_THRESHOLD) {
        rssi = rtl::tools::calculateRssiDirectSampling(bufferQ_.data(), samples);
    } else {
        rssi = rtl::tools::calculateRssi(fft_power_sum, groups_num);
    }

    spdlog::debug("Scan Freq: {} MHz, RSSI: {} dBFS", centerFreq / 1e6, rssi);
    std::vector<double> spectrum_db = rtl::tools::spectrumToDb(fft_power_sum, groups_num);
    rtl::tools::suppressDcSpike(spectrum_db);
    // Detector budgets keep expensive IQ feature checks bounded per sweep while
    // still allowing strong candidates from multiple hops through.
    if (currentFmIqBudget_ > 0) {
        int  attempts   = 0;
        auto detections = fmDetector_.detectInIqSegment(
            spectrum_db,
            static_cast<double>(centerFreq) - static_cast<double>(activeSampleRateHz_) / 2.0,
            static_cast<double>(centerFreq) + static_cast<double>(activeSampleRateHz_) / 2.0,
            bufferU8_.data(),
            n_read,
            static_cast<double>(centerFreq),
            static_cast<double>(activeSampleRateHz_),
            currentFmIqBudget_,
            &attempts);
        currentFmIqBudget_ -= std::min(currentFmIqBudget_, attempts);
        currentFmDetections_.insert(currentFmDetections_.end(), detections.begin(), detections.end());
    }
    if (currentAmIqBudget_ > 0) {
        int  attempts     = 0;
        auto amDetections = amDetector_.detectInIqSegment(
            spectrum_db,
            static_cast<double>(centerFreq) - static_cast<double>(activeSampleRateHz_) / 2.0,
            static_cast<double>(centerFreq) + static_cast<double>(activeSampleRateHz_) / 2.0,
            bufferU8_.data(),
            n_read,
            static_cast<double>(centerFreq),
            static_cast<double>(activeSampleRateHz_),
            currentAmIqBudget_,
            &attempts);
        currentAmIqBudget_ -= std::min(currentAmIqBudget_, attempts);
        currentAmDetections_.insert(currentAmDetections_.end(), amDetections.begin(), amDetections.end());
    }
    segments.push_back({std::move(spectrum_db), static_cast<double>(centerFreq)});
    return true;
}

void ScanEngine::spliceAndPush(
    const std::vector<SegmentData>& segments,
    std::uint32_t                   sweepStartFreq,
    std::uint32_t                   sweepEndFreq,
    const std::function<bool()>&    shouldContinue) {
    auto [spliced_spectrum, spliced_freqs] = rtl::tools::spliceSpectrum(
        segments,
        static_cast<double>(activeSampleRateHz_),
        static_cast<double>(sweepStartFreq),
        static_cast<double>(sweepEndFreq));

    rtl::tools::suppressPeriodicSpurs(
        spliced_spectrum, static_cast<double>(sweepStartFreq), static_cast<double>(sweepEndFreq));

    // Smooth in linear power, not dB, so averaging preserves physical power
    // relationships and avoids biasing very weak bins.
    constexpr double EMA_ALPHA = 0.6;
    if (hasPrevSpectrum_ && prevSpectrum_.size() == spliced_spectrum.size()) {
        for (std::size_t i = 0; i < spliced_spectrum.size(); ++i) {
            double p_prev = std::pow(10.0, prevSpectrum_[i] / 10.0);
            double p_curr = std::pow(10.0, spliced_spectrum[i] / 10.0);
            double p_avg  = EMA_ALPHA * p_prev + (1.0 - EMA_ALPHA) * p_curr;
            spliced_spectrum[i] = 10.0 * std::log10(std::max(p_avg, 1e-30));
        }
    }
    prevSpectrum_     = spliced_spectrum;
    hasPrevSpectrum_  = true;

    nlohmann::json result_arr = nlohmann::json::array();
    // Merge overlapping detections from adjacent hops, then use short-lived
    // tracks to suppress one-sweep flicker in the published result list.
    std::sort(currentFmDetections_.begin(), currentFmDetections_.end(), [](const auto& a, const auto& b) {
        if (a.centerHz == b.centerHz) return a.confidence > b.confidence;
        return a.centerHz < b.centerHz;
    });
    std::vector<FmDetection> deduped;
    for (const auto& detection : currentFmDetections_) {
        if (!deduped.empty() && std::abs(deduped.back().centerHz - detection.centerHz) < 120000.0) {
            if (detection.confidence > deduped.back().confidence) { deduped.back() = detection; }
            continue;
        }
        deduped.push_back(detection);
    }
    deduped = updateFmTracks(std::move(deduped));

    for (const auto& detection : deduped) {
        nlohmann::json item;
        item["type"]         = "fm_spec";
        item["cf"]           = detection.centerHz;
        item["start_freq"]   = detection.startFreqHz;
        item["end_freq"]     = detection.endFreqHz;
        item["bw"]           = detection.bandwidthHz;
        item["peak_db"]      = detection.peakDb;
        item["avg_db"]       = detection.avgDb;
        item["noise_db"]     = detection.noiseDb;
        item["snr_db"]       = detection.snrDb;
        item["confidence"]   = detection.confidence;
        item["verified"]     = detection.verified;
        item["stereo"]       = detection.stereo;
        item["rds"]          = detection.rds;
        item["pilot_snr_db"] = detection.pilotSnrDb;
        item["audio_snr_db"] = detection.audioSnrDb;
        item["deviation_hz"] = detection.deviationHz;
        item["papr_db"]      = detection.paprDb;
        item["am_variance"]  = detection.amVariance;
        item["fm_rms_hz"]    = detection.fmRmsHz;
        result_arr.push_back(std::move(item));
    }

    std::sort(currentAmDetections_.begin(), currentAmDetections_.end(), [](const auto& a, const auto& b) {
        if (a.centerHz == b.centerHz) return a.confidence > b.confidence;
        return a.centerHz < b.centerHz;
    });
    std::vector<AmDetection> dedupedAm;
    for (const auto& detection : currentAmDetections_) {
        if (!dedupedAm.empty() && std::abs(dedupedAm.back().centerHz - detection.centerHz) < 30000.0) {
            if (detection.confidence > dedupedAm.back().confidence) { dedupedAm.back() = detection; }
            continue;
        }
        dedupedAm.push_back(detection);
    }
    dedupedAm = updateAmTracks(std::move(dedupedAm));

    for (const auto& detection : dedupedAm) {
        nlohmann::json item;
        item["type"]             = "am_spec";
        item["cf"]               = detection.centerHz;
        item["start_freq"]       = detection.startFreqHz;
        item["end_freq"]         = detection.endFreqHz;
        item["bw"]               = detection.bandwidthHz;
        item["peak_db"]          = detection.peakDb;
        item["avg_db"]           = detection.avgDb;
        item["noise_db"]         = detection.noiseDb;
        item["snr_db"]           = detection.snrDb;
        item["confidence"]       = detection.confidence;
        item["verified"]         = detection.verified;
        item["audio_snr_db"]     = detection.audioSnrDb;
        item["modulation_depth"] = detection.modulationDepth;
        item["carrier_snr_db"]   = detection.carrierSnrDb;
        item["fm_rms_hz"]        = detection.fmRmsHz;
        result_arr.push_back(std::move(item));
    }

    double max_val = *std::max_element(spliced_spectrum.begin(), spliced_spectrum.end());
    double min_val = *std::min_element(spliced_spectrum.begin(), spliced_spectrum.end());

    nlohmann::json data_obj;
    data_obj["start_freq"] = sweepStartFreq;
    data_obj["end_freq"]   = sweepEndFreq;
    data_obj["max_value"]  = max_val;
    data_obj["min_value"]  = min_val;
    data_obj["data"]       = spliced_spectrum;
    data_obj["result"]     = result_arr;

    nlohmann::json json_data;
    json_data["id"]    = 200;
    json_data["event"] = "scan_data";
    json_data["data"]  = data_obj;

    pusher_.pushScanHop(std::move(json_data));

    spdlog::info("Sweep complete, {} bins pushed", spliced_spectrum.size());
}

std::vector<FmDetection> ScanEngine::updateFmTracks(std::vector<FmDetection> detections) {
    constexpr double TRACK_MATCH_HZ       = 180000.0;
    constexpr double NEW_TRACK_CONFIDENCE = 0.40;
    constexpr int    SHOW_HITS            = 1;
    constexpr int    DROP_MISSES          = 5;

    std::vector<char> matched(fmTracks_.size(), 0);

    for (const auto& detection : detections) {
        int    bestIndex = -1;
        double bestDist  = TRACK_MATCH_HZ;
        for (int i = 0; i < static_cast<int>(fmTracks_.size()); ++i) {
            const double dist =
                std::abs(fmTracks_[static_cast<std::size_t>(i)].detection.centerHz - detection.centerHz);
            if (dist < bestDist) {
                bestDist  = dist;
                bestIndex = i;
            }
        }

        if (bestIndex >= 0) {
            auto& track = fmTracks_[static_cast<std::size_t>(bestIndex)];
            smoothDetection(track.detection, detection);
            track.hits    += 1;
            track.misses   = 0;
            track.visible  = track.visible || track.hits >= SHOW_HITS || track.detection.confidence >= 0.50;
            matched[static_cast<std::size_t>(bestIndex)] = 1;
        } else if (detection.confidence >= NEW_TRACK_CONFIDENCE) {
            FmTrack track;
            track.detection = detection;
            track.hits      = 1;
            track.misses    = 0;
            track.visible   = detection.confidence >= 0.50;
            fmTracks_.push_back(track);
            matched.push_back(1);
        }
    }

    for (std::size_t i = 0; i < fmTracks_.size(); ++i) {
        if (i < matched.size() && matched[i]) continue;
        fmTracks_[i].misses               += 1;
        fmTracks_[i].detection.confidence  = std::max(fmTracks_[i].detection.confidence - 0.05, 0.0);
    }

    fmTracks_.erase(
        std::remove_if(
            fmTracks_.begin(),
            fmTracks_.end(),
            [](const FmTrack& track) {
                return track.misses >= DROP_MISSES || track.detection.confidence < 0.20;
            }),
        fmTracks_.end());

    std::vector<FmDetection> stable;
    for (const auto& track : fmTracks_) {
        if (track.visible && track.misses < DROP_MISSES) stable.push_back(track.detection);
    }
    std::sort(stable.begin(), stable.end(), [](const auto& a, const auto& b) {
        return a.centerHz < b.centerHz;
    });
    return stable;
}

std::vector<AmDetection> ScanEngine::updateAmTracks(std::vector<AmDetection> detections) {
    constexpr double TRACK_MATCH_HZ       = 35000.0;
    constexpr double NEW_TRACK_CONFIDENCE = 0.38;
    constexpr int    SHOW_HITS            = 1;
    constexpr int    DROP_MISSES          = 4;

    std::vector<char> matched(amTracks_.size(), 0);

    for (const auto& detection : detections) {
        int    bestIndex = -1;
        double bestDist  = TRACK_MATCH_HZ;
        for (int i = 0; i < static_cast<int>(amTracks_.size()); ++i) {
            const double dist =
                std::abs(amTracks_[static_cast<std::size_t>(i)].detection.centerHz - detection.centerHz);
            if (dist < bestDist) {
                bestDist  = dist;
                bestIndex = i;
            }
        }

        if (bestIndex >= 0) {
            auto& track = amTracks_[static_cast<std::size_t>(bestIndex)];
            smoothDetection(track.detection, detection);
            track.hits   += 1;
            track.misses  = 0;
            track.visible = track.visible || track.hits >= SHOW_HITS || track.detection.confidence >= 0.48;
            matched[static_cast<std::size_t>(bestIndex)] = 1;
        } else if (detection.confidence >= NEW_TRACK_CONFIDENCE) {
            AmTrack track;
            track.detection = detection;
            track.hits      = 1;
            track.misses    = 0;
            track.visible   = detection.confidence >= 0.48;
            amTracks_.push_back(track);
            matched.push_back(1);
        }
    }

    for (std::size_t i = 0; i < amTracks_.size(); ++i) {
        if (i < matched.size() && matched[i]) continue;
        amTracks_[i].misses              += 1;
        amTracks_[i].detection.confidence = std::max(amTracks_[i].detection.confidence - 0.06, 0.0);
    }

    amTracks_.erase(
        std::remove_if(
            amTracks_.begin(),
            amTracks_.end(),
            [](const AmTrack& track) {
                return track.misses >= DROP_MISSES || track.detection.confidence < 0.20;
            }),
        amTracks_.end());

    std::vector<AmDetection> stable;
    for (const auto& track : amTracks_) {
        if (track.visible && track.misses < DROP_MISSES) stable.push_back(track.detection);
    }
    std::sort(stable.begin(), stable.end(), [](const auto& a, const auto& b) {
        return a.centerHz < b.centerHz;
    });
    return stable;
}

} // namespace rtl::scanner
