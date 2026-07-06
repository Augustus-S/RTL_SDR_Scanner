#include "scanner/fm_detector.hpp"
#include "constants.hpp"
#include <algorithm>
#include <cmath>
#include <complex>
#include <numeric>
#include <fftw3.h>
#include <spdlog/spdlog.h>

/**
 * @file fm_detector.cpp
 * @brief FM broadcast detector using spectrum candidates and IQ feature checks.
 */

namespace rtl::scanner {

namespace {

double clamp01(double value) {
    return std::clamp(value, 0.0, 1.0);
}

double median(std::vector<double> values) {
    if (values.empty()) return -200.0;
    const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), mid, values.end());
    return *mid;
}

double meanPowerDb(const std::vector<double>& values, int first, int last) {
    if (values.empty() || first > last) return -200.0;
    first = std::max(first, 0);
    last  = std::min(last, static_cast<int>(values.size()) - 1);
    if (first > last) return -200.0;

    double sum = 0.0;
    int    n   = 0;
    for (int i = first; i <= last; ++i) {
        sum += std::pow(10.0, values[static_cast<std::size_t>(i)] / 10.0);
        ++n;
    }
    if (n == 0) return -200.0;
    return 10.0 * std::log10(std::max(sum / n, 1e-20));
}

double bandPowerDb(const std::vector<double>& spectrum, double sampleRate, double loHz, double hiHz) {
    if (spectrum.empty() || hiHz <= loHz) return -200.0;
    const double binHz = sampleRate / static_cast<double>(spectrum.size() * 2);
    int          first = static_cast<int>(std::ceil(loHz / binHz));
    int          last  = static_cast<int>(std::floor(hiHz / binHz));
    return meanPowerDb(spectrum, first, last);
}

double peakSnrDb(
    const std::vector<double>& spectrum, double sampleRate, double freqHz, double halfWidthHz, double guardHz) {
    if (spectrum.empty()) return 0.0;
    const double binHz     = sampleRate / static_cast<double>(spectrum.size() * 2);
    int          peakFirst = static_cast<int>(std::floor((freqHz - halfWidthHz) / binHz));
    int          peakLast  = static_cast<int>(std::ceil((freqHz + halfWidthHz) / binHz));
    peakFirst              = std::max(peakFirst, 0);
    peakLast               = std::min(peakLast, static_cast<int>(spectrum.size()) - 1);
    if (peakFirst > peakLast) return 0.0;

    double peak = -200.0;
    for (int i = peakFirst; i <= peakLast; ++i) { peak = std::max(peak, spectrum[static_cast<std::size_t>(i)]); }

    std::vector<double> noise;
    int                 leftFirst  = static_cast<int>(std::floor((freqHz - guardHz - 2.0 * halfWidthHz) / binHz));
    int                 leftLast   = static_cast<int>(std::floor((freqHz - guardHz) / binHz));
    int                 rightFirst = static_cast<int>(std::ceil((freqHz + guardHz) / binHz));
    int                 rightLast  = static_cast<int>(std::ceil((freqHz + guardHz + 2.0 * halfWidthHz) / binHz));
    for (int i = leftFirst; i <= leftLast; ++i) {
        if (i >= 0 && i < static_cast<int>(spectrum.size())) noise.push_back(spectrum[static_cast<std::size_t>(i)]);
    }
    for (int i = rightFirst; i <= rightLast; ++i) {
        if (i >= 0 && i < static_cast<int>(spectrum.size())) noise.push_back(spectrum[static_cast<std::size_t>(i)]);
    }
    const double noiseDb = median(noise);
    return peak - noiseDb;
}

std::vector<double> makeHannSpectrum(const std::vector<double>& samples, std::uint32_t sampleRate, int fftSize) {
    (void)sampleRate;
    struct FftCache {
        int                 fftSize = 0;
        std::vector<double> window;
        double              windowPower = 0.0;
        double*             in          = nullptr;
        fftw_complex*       out         = nullptr;
        fftw_plan           plan        = nullptr;

        ~FftCache() {
            if (plan) fftw_destroy_plan(plan);
            fftw_free(in);
            fftw_free(out);
        }

        bool ensure(int size) {
            if (fftSize == size && plan && in && out) return true;
            if (plan) fftw_destroy_plan(plan);
            fftw_free(in);
            fftw_free(out);
            plan        = nullptr;
            in          = nullptr;
            out         = nullptr;
            fftSize     = size;
            windowPower = 0.0;
            window.resize(static_cast<std::size_t>(fftSize));
            for (int i = 0; i < fftSize; ++i) {
                window[static_cast<std::size_t>(i)] = 0.5 * (1.0 - std::cos(2.0 * M_PI * i / (fftSize - 1)));
                windowPower += window[static_cast<std::size_t>(i)] * window[static_cast<std::size_t>(i)];
            }
            in  = static_cast<double*>(fftw_malloc(sizeof(double) * fftSize));
            out = static_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * (fftSize / 2 + 1)));
            if (!in || !out) return false;
            plan = fftw_plan_dft_r2c_1d(fftSize, in, out, FFTW_ESTIMATE);
            return plan != nullptr;
        }
    };

    std::vector<double> spectrum(static_cast<std::size_t>(fftSize / 2), -200.0);
    if (static_cast<int>(samples.size()) < fftSize) return spectrum;

    thread_local FftCache cache;
    if (!cache.ensure(fftSize)) return spectrum;

    const int           blocks = std::min(32, static_cast<int>(samples.size()) / fftSize);
    std::vector<double> power(static_cast<std::size_t>(fftSize / 2), 0.0);
    for (int b = 0; b < blocks; ++b) {
        for (int n = 0; n < fftSize; ++n) {
            cache.in[n] =
                samples[static_cast<std::size_t>(b * fftSize + n)] * cache.window[static_cast<std::size_t>(n)];
        }
        fftw_execute(cache.plan);
        for (int k = 0; k < fftSize / 2; ++k) {
            const double re                     = cache.out[k][0];
            const double im                     = cache.out[k][1];
            power[static_cast<std::size_t>(k)] += re * re + im * im;
        }
    }

    const double norm = std::max(cache.windowPower * blocks, 1e-12);
    for (int k = 0; k < fftSize / 2; ++k) {
        spectrum[static_cast<std::size_t>(k)] =
            10.0 * std::log10(std::max(power[static_cast<std::size_t>(k)] / norm, 1e-20));
    }

    return spectrum;
}

} // namespace

FMDetector::FMDetector(FmDetectionConfig config)
    : config_(config) {}

std::vector<FmCandidate>
    FMDetector::findCandidates(const std::vector<double>& spectrum, double sweepStartHz, double sweepEndHz) const {
    std::vector<FmCandidate> candidates;
    const int                n = static_cast<int>(spectrum.size());
    if (n < 8 || sweepEndHz <= sweepStartHz) return candidates;

    // Restrict candidate search to the broadcast FM band even when a scan range
    // spans other services.
    const double detectStartHz = std::max<double>(sweepStartHz, config_.fmBandMinHz);
    const double detectEndHz   = std::min<double>(sweepEndHz, config_.fmBandMaxHz);
    if (detectEndHz <= detectStartHz) return candidates;

    const double binHz    = (sweepEndHz - sweepStartHz) / static_cast<double>(n - 1);
    const int    firstBin = std::max(0, static_cast<int>(std::floor((detectStartHz - sweepStartHz) / binHz)));
    const int    lastBin  = std::min(n - 1, static_cast<int>(std::ceil((detectEndHz - sweepStartHz) / binHz)));
    if (lastBin <= firstBin) return candidates;

    // Wide FM stations occupy many adjacent bins. Smooth before thresholding and
    // merge short inactive gaps so multipath/notches do not split one station.
    std::vector<double> smoothed(spectrum);
    for (int i = firstBin + 2; i <= lastBin - 2; ++i) {
        smoothed[static_cast<std::size_t>(i)] =
            (spectrum[static_cast<std::size_t>(i - 2)] + 2.0 * spectrum[static_cast<std::size_t>(i - 1)]
             + 3.0 * spectrum[static_cast<std::size_t>(i)] + 2.0 * spectrum[static_cast<std::size_t>(i + 1)]
             + spectrum[static_cast<std::size_t>(i + 2)])
            / 9.0;
    }

    std::vector<double> bandValues;
    bandValues.reserve(static_cast<std::size_t>(lastBin - firstBin + 1));
    for (int i = firstBin; i <= lastBin; ++i) {
        const double value = smoothed[static_cast<std::size_t>(i)];
        if (std::isfinite(value) && value > -180.0) bandValues.push_back(value);
    }
    const double globalNoise = median(bandValues);

    auto localNoise = [&](int bin) {
        const int           radius = std::max(20, static_cast<int>(std::round(1000000.0 / binHz)));
        const int           guard  = std::max(4, static_cast<int>(std::round(180000.0 / binHz)));
        std::vector<double> values;
        for (int i = bin - radius; i <= bin + radius; ++i) {
            if (i < firstBin || i > lastBin || std::abs(i - bin) <= guard) continue;
            const double value = smoothed[static_cast<std::size_t>(i)];
            if (std::isfinite(value) && value > -180.0) values.push_back(value);
        }
        return values.empty() ? globalNoise : median(values);
    };

    std::vector<char> active(static_cast<std::size_t>(n), 0);
    for (int i = firstBin; i <= lastBin; ++i) {
        const double noise = localNoise(i);
        if (smoothed[static_cast<std::size_t>(i)] >= noise + config_.candidateThresholdDb) {
            active[static_cast<std::size_t>(i)] = 1;
        }
    }

    const int mergeGapBins = std::max(1, static_cast<int>(std::round(35000.0 / binHz)));
    int       i            = firstBin;
    while (i <= lastBin) {
        while (i <= lastBin && !active[static_cast<std::size_t>(i)]) ++i;
        if (i > lastBin) break;

        int start = i;
        int stop  = i;
        int gap   = 0;
        ++i;
        while (i <= lastBin) {
            if (active[static_cast<std::size_t>(i)]) {
                stop = i;
                gap  = 0;
            } else if (++gap > mergeGapBins) {
                break;
            }
            ++i;
        }

        const double bandwidthHz = (stop - start + 1) * binHz;
        if (bandwidthHz < config_.minCandidateBwHz || bandwidthHz > config_.maxCandidateBwHz) continue;

        double peak = -200.0;
        double sum  = 0.0;
        for (int j = start; j <= stop; ++j) {
            const double value  = spectrum[static_cast<std::size_t>(j)];
            peak                = std::max(peak, value);
            sum                += value;
        }
        const double avg     = sum / static_cast<double>(stop - start + 1);
        const double noise   = localNoise((start + stop) / 2);
        const double snr     = peak - noise;
        const double center  = sweepStartHz + ((start + stop) * 0.5) * binHz;
        const double startHz = sweepStartHz + start * binHz;
        const double endHz   = sweepStartHz + stop * binHz;
        if (snr < config_.minSnrDb) continue;

        FmCandidate candidate;
        candidate.startFreqHz = startHz;
        candidate.endFreqHz   = endHz;
        candidate.centerHz    = center;
        candidate.bandwidthHz = bandwidthHz;
        candidate.peakDb      = peak;
        candidate.avgDb       = avg;
        candidate.noiseDb     = noise;
        candidate.snrDb       = snr;
        const double bwScore  = 1.0 - std::min(std::abs(bandwidthHz - 200000.0) / 180000.0, 1.0);
        const double snrScore = clamp01((snr - config_.minSnrDb) / 18.0);
        candidate.confidence  = clamp01(0.25 + 0.35 * bwScore + 0.40 * snrScore);
        candidates.push_back(candidate);
    }

    std::sort(candidates.begin(), candidates.end(), [](const FmCandidate& a, const FmCandidate& b) {
        if (a.confidence == b.confidence) return a.snrDb > b.snrDb;
        return a.confidence > b.confidence;
    });

    return candidates;
}

std::vector<FmDetection> FMDetector::verifyCandidates(
    PersistentAsyncReader&          reader,
    const std::vector<FmCandidate>& candidates,
    double                          sampleRateHz,
    const std::function<bool()>&    shouldContinue) const {
    std::vector<FmDetection> detections;
    int                      verifiedCount = 0;
    if (sampleRateHz <= 0.0) return detections;

    for (const auto& candidate : candidates) {
        if (shouldContinue && !shouldContinue()) break;
        if (verifiedCount >= config_.maxIqVerificationsPerSweep) break;

        FmDetection detection;
        detection.startFreqHz = candidate.startFreqHz;
        detection.endFreqHz   = candidate.endFreqHz;
        detection.centerHz    = candidate.centerHz;
        detection.bandwidthHz = candidate.bandwidthHz;
        detection.peakDb      = candidate.peakDb;
        detection.avgDb       = candidate.avgDb;
        detection.noiseDb     = candidate.noiseDb;
        detection.snrDb       = candidate.snrDb;
        detection.confidence  = candidate.confidence;

        const std::uint32_t readLen =
            static_cast<std::uint32_t>(sampleRateHz * 2.0 * config_.iqVerifyDurationMs / 1000.0);
        std::vector<std::uint8_t> iq(readLen);
        std::uint32_t             outLen = readLen;
        const auto                tuneHz = static_cast<std::uint32_t>(std::clamp(
            candidate.centerHz + config_.verifyTuningOffsetHz,
            static_cast<double>(rtl::constants::MIN_FREQ),
            static_cast<double>(rtl::constants::MAX_FREQ)));
        auto                      result = reader.read(
            iq.data(),
            &outLen,
            tuneHz,
            0,
            rtl::constants::READ_TIMEOUT_MS,
            50,
            ResetPolicy::ALWAYS,
            true);
        ++verifiedCount;

        if (result == PersistentAsyncReader::ReadResult::SUCCESS) {
            auto features = analyzeIq(
                iq.data(),
                outLen,
                static_cast<double>(config_.verifyTuningOffsetHz),
                sampleRateHz);
            const bool strongEnoughWeakPilot =
                features.pilotSnrDb >= config_.weakPilotSnrDb && candidate.snrDb >= config_.strongSpectrumSnrDb;
            const bool featureAccepted =
                features.audioSnrDb >= config_.minAudioSnrDb || features.pilotSnrDb >= config_.minPilotSnrDb
                || strongEnoughWeakPilot;
            if (features.valid && featureAccepted) {
                detection.verified    = true;
                detection.stereo      = features.stereo;
                detection.rds         = features.rds;
                detection.pilotSnrDb  = features.pilotSnrDb;
                detection.audioSnrDb  = features.audioSnrDb;
                detection.deviationHz = features.deviationHz;
                detection.paprDb      = features.paprDb;
                detection.amVariance  = features.amVariance;
                detection.fmRmsHz     = features.fmRmsHz;
                detection.confidence  = clamp01(0.45 * candidate.confidence + 0.55 * features.confidence);
            }
        } else {
            spdlog::warn("FM IQ verification read failed near {:.3f} MHz", candidate.centerHz / 1e6);
        }

        if (detection.verified) { detections.push_back(detection); }
    }

    return detections;
}

std::vector<FmDetection> FMDetector::detectInIqSegment(
    const std::vector<double>& spectrum,
    double                     segmentStartHz,
    double                     segmentEndHz,
    const std::uint8_t*        iq,
    std::uint32_t              bytesRead,
    double                     tunerCenterHz,
    double                     sampleRateHz,
    int                        maxIqVerifications,
    int*                       verificationAttempts) const {
    std::vector<FmDetection> detections;
    if (verificationAttempts) *verificationAttempts = 0;
    if (!iq || bytesRead < 4096 || maxIqVerifications <= 0) return detections;

    // Fast path used by ScanEngine: verify candidates against the IQ already
    // captured for this hop instead of spending extra retunes per candidate.
    auto      candidates = findCandidates(spectrum, segmentStartHz, segmentEndHz);
    const int limit      = std::min<int>(maxIqVerifications, static_cast<int>(candidates.size()));
    if (verificationAttempts) *verificationAttempts = limit;
    for (int i = 0; i < limit; ++i) {
        const auto& candidate     = candidates[static_cast<std::size_t>(i)];
        const auto  mixerOffsetHz = tunerCenterHz - candidate.centerHz;
        auto        features      = analyzeIq(iq, bytesRead, mixerOffsetHz, sampleRateHz);
        if (!features.valid) continue;
        const bool strongEnoughWeakPilot =
            features.pilotSnrDb >= config_.weakPilotSnrDb && candidate.snrDb >= config_.strongSpectrumSnrDb;
        const bool featureAccepted =
            features.audioSnrDb >= config_.minAudioSnrDb || features.pilotSnrDb >= config_.minPilotSnrDb
            || strongEnoughWeakPilot;
        if (!featureAccepted) continue;

        FmDetection detection;
        detection.startFreqHz = candidate.startFreqHz;
        detection.endFreqHz   = candidate.endFreqHz;
        detection.centerHz    = candidate.centerHz;
        detection.bandwidthHz = candidate.bandwidthHz;
        detection.peakDb      = candidate.peakDb;
        detection.avgDb       = candidate.avgDb;
        detection.noiseDb     = candidate.noiseDb;
        detection.snrDb       = candidate.snrDb;
        detection.confidence  = clamp01(0.45 * candidate.confidence + 0.55 * features.confidence);
        detection.verified    = true;
        detection.stereo      = features.stereo;
        detection.rds         = features.rds;
        detection.pilotSnrDb  = features.pilotSnrDb;
        detection.audioSnrDb  = features.audioSnrDb;
        detection.deviationHz = features.deviationHz;
        detection.paprDb      = features.paprDb;
        detection.amVariance  = features.amVariance;
        detection.fmRmsHz     = features.fmRmsHz;
        detections.push_back(detection);
    }
    return detections;
}

FMDetector::IqFeatures
    FMDetector::analyzeIq(
        const std::uint8_t* iq, std::uint32_t bytesRead, double mixerOffsetHz, double sampleRateHz) const {
    IqFeatures features;
    if (bytesRead < 4096 || !iq) return features;

    const std::uint32_t samples = bytesRead / 2;
    std::vector<double> composite;
    composite.reserve(samples > 0 ? samples - 1 : 0);

    // Mix the candidate to baseband, run a quadrature discriminator, then look
    // for audio energy, 19 kHz pilot, optional RDS, and FM-like deviation.
    const double mixerPhaseInc = 2.0 * M_PI * mixerOffsetHz / sampleRateHz;
    double       oscI          = 1.0;
    double       oscQ          = 0.0;
    const double stepI         = std::cos(mixerPhaseInc);
    const double stepQ         = std::sin(mixerPhaseInc);

    bool   havePrev = false;
    double prevI    = 0.0;
    double prevQ    = 0.0;
    double sumSq    = 0.0;
    double ampSum   = 0.0;
    double ampSqSum = 0.0;
    double ampPeak  = 0.0;
    int    ampCount = 0;

    for (std::uint32_t i = 0; i + 1 < bytesRead; i += 2) {
        const double rawI  = (static_cast<double>(iq[static_cast<std::size_t>(i)]) - 127.5) / 127.5;
        const double rawQ  = (static_cast<double>(iq[static_cast<std::size_t>(i + 1)]) - 127.5) / 127.5;
        const double curI  = rawI * oscI - rawQ * oscQ;
        const double curQ  = rawI * oscQ + rawQ * oscI;
        const double amp2  = curI * curI + curQ * curQ;
        const double amp   = std::sqrt(std::max(amp2, 0.0));
        ampSum            += amp;
        ampSqSum          += amp2;
        ampPeak            = std::max(ampPeak, amp2);
        ++ampCount;

        const double nextOscI = oscI * stepI - oscQ * stepQ;
        const double nextOscQ = oscI * stepQ + oscQ * stepI;
        oscI                  = nextOscI;
        oscQ                  = nextOscQ;

        if (!havePrev) {
            prevI    = curI;
            prevQ    = curQ;
            havePrev = true;
            continue;
        }

        const double imag  = curQ * prevI - curI * prevQ;
        const double power = amp2 + 1e-12;
        const double disc  = imag / power;
        prevI              = curI;
        prevQ              = curQ;
        composite.push_back(disc);
        sumSq += disc * disc;
    }

    if (composite.size() < 4096) return features;

    const double rms     = std::sqrt(sumSq / static_cast<double>(composite.size()));
    features.fmRmsHz     = rms * sampleRateHz / (2.0 * M_PI);
    features.deviationHz = features.fmRmsHz;
    if (ampCount > 0 && ampSqSum > 0.0) {
        const double meanAmp  = ampSum / static_cast<double>(ampCount);
        const double meanAmp2 = ampSqSum / static_cast<double>(ampCount);
        const double ampVar   = std::max(meanAmp2 - meanAmp * meanAmp, 0.0);
        features.amVariance   = ampVar / std::max(meanAmp2, 1e-12);
        features.paprDb       = 10.0 * std::log10(std::max(ampPeak / std::max(meanAmp2, 1e-12), 1e-12));
    }

    constexpr int fftSize  = 2048;
    auto          spectrum = makeHannSpectrum(composite, static_cast<std::uint32_t>(sampleRateHz), fftSize);
    const double  audioDb  = bandPowerDb(spectrum, sampleRateHz, 300.0, 15000.0);
    const double  refDb    = bandPowerDb(spectrum, sampleRateHz, 70000.0, 100000.0);
    features.audioSnrDb    = audioDb - refDb;
    features.pilotSnrDb    = peakSnrDb(spectrum, sampleRateHz, config_.pilotFreqHz, 600.0, 2200.0);
    features.rdsSnrDb      = peakSnrDb(spectrum, sampleRateHz, config_.rdsFreqHz, 1200.0, 3500.0);
    features.stereo        = features.pilotSnrDb >= config_.minPilotSnrDb;
    features.rds           = features.rdsSnrDb >= config_.minPilotSnrDb;

    const bool audioOk     = features.audioSnrDb >= config_.minAudioSnrDb;
    const bool pilotOk     = features.pilotSnrDb >= config_.minPilotSnrDb;
    const bool weakPilotOk = features.pilotSnrDb >= config_.weakPilotSnrDb;
    if (!audioOk && !pilotOk && !weakPilotOk) return features;

    const double audioScore     = clamp01((features.audioSnrDb - config_.minAudioSnrDb) / 16.0);
    const double pilotScore     = clamp01((features.pilotSnrDb - config_.minPilotSnrDb) / 14.0);
    const double devScore       = clamp01(1.0 - std::abs(features.deviationHz - 75000.0) / 90000.0);
    const double weakPilotScore = clamp01((features.pilotSnrDb - config_.weakPilotSnrDb) / 8.0);
    const double amPenalty      = clamp01((features.amVariance - 0.30) / 0.70) * 0.10;
    features.confidence =
        clamp01(0.30 + 0.25 * audioScore + 0.25 * pilotScore + 0.12 * weakPilotScore + 0.08 * devScore - amPenalty);
    features.valid = true;
    return features;
}

} // namespace rtl::scanner
